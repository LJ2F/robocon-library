#include "stepper_dma_control.hpp"

#include <algorithm>

namespace gdut {
//构造函数
multi_stepper_dma::multi_stepper_dma(
    gdut::timer &timer,
    const std::array<motor_config, motor_count> &configs)
    : m_timer(timer), m_timer_proxy(&timer), m_oc(&timer), m_cfg(configs) {}

    //初始化函数
HAL_StatusTypeDef multi_stepper_dma::init() {
  if (m_timer.get_htim() == nullptr) {
    return HAL_ERROR;
  }

  // 注册 4 个通道的 CC 回调
  bool ok = true;
  ok &= m_timer.register_capture_callback(1, [this]() { on_cc_event(0); });
  ok &= m_timer.register_capture_callback(2, [this]() { on_cc_event(1); });
   ok &= m_timer.register_capture_callback(3, [this]() { on_cc_event(2); });
  ok &= m_timer.register_capture_callback(4, [this]() { on_cc_event(3); });
  if (!ok) {
    return HAL_ERROR;
  }

  // 初始化各路 DMA，并分别绑定各自的完成/错误回调。
  // 这里不调用 timer.attach_dma()，因为 timer 只有一个 DMA 槽位。
  for (std::size_t i = 0; i < motor_count; ++i) {
    auto &cfg = m_cfg[i];//每个电机一个
    const bool unused_cfg = (cfg.dma == nullptr && cfg.dir_port == nullptr &&
                             cfg.dir_pin == 0U && cfg.pulse_high_ticks == 0U);
    if (unused_cfg) {
      m_state[i] = {};
      continue;
    }

    if (cfg.dma == nullptr || cfg.dir_port == nullptr || cfg.dir_pin == 0U ||
        cfg.pulse_high_ticks == 0U) {
      return HAL_ERROR;
    }

    cfg.dma->init();
    cfg.dma->set_callback_handler([this, i](std::error_code ec) {
      if (ec) {
        on_dma_error(i);//每个dma的错误回调
      } else {
        on_dma_complete(i);//每个dma的完成回调
      }
    });

    m_state[i] = {};
  }

  // 启动 timer 基本计数，HAL_TIM_BASIC_START
  if (m_timer.start() != HAL_OK) {
    return HAL_ERROR;
  }

#if defined(TIM_BDTR_MOE)
  if (IS_TIM_BREAK_INSTANCE(m_timer.get_htim()->Instance)) {
    __HAL_TIM_MOE_ENABLE(m_timer.get_htim());
  }
#endif

  return HAL_OK;
}
//是否正在运行
bool multi_stepper_dma::is_busy(std::size_t motor_id) const {
  return (motor_id < motor_count) ? m_state[motor_id].busy : false;
}
//dma的停止
bool multi_stepper_dma::has_dma_error(std::size_t motor_id) const {
  return (motor_id < motor_count) ? m_state[motor_id].dma_error : true;
}

//构建表值
std::size_t multi_stepper_dma::build_toggle_table(const uint16_t *period_table,//每个脉冲的总周期表
                                                  std::size_t pulse_count,  //脉冲数
                                                  uint16_t pulse_high_ticks,//脉冲高时间
                                                  uint32_t first_rise_tick,//第一个上升沿时间
                                                  uint32_t *out_toggle_table,//输出数组，保存所有翻转的值
                                                  std::size_t out_cap) //输出数组容量
                                                  {
  if (period_table == nullptr || out_toggle_table == nullptr || pulse_count == 0U) {
    return 0U;
  }
//输出数组容量是否足够
  if (out_cap < pulse_count * 2U) {
    return 0U;
  }

  uint32_t t = first_rise_tick;
  std::size_t idx = 0U;
//脉冲数
  for (std::size_t i = 0; i < pulse_count; ++i) {
    const uint16_t period = period_table[i];
    if (period <= pulse_high_ticks) {
      return 0U;
    }

    // 上升沿
    out_toggle_table[idx++] = t;

    // 下降沿
    t += pulse_high_ticks;
    out_toggle_table[idx++] = t;

    // 下一脉冲上升沿
    t += (period - pulse_high_ticks);
  }

  return idx;
}
//开始移动
HAL_StatusTypeDef multi_stepper_dma::start_motor(std::size_t motor_id,//那一路的dma
                                                 const uint32_t *toggle_table,//每个翻转的时间表
                                                 std::size_t toggle_count,//翻转次数
                                                 GPIO_PinState dir_level,//方向
                                                 uint32_t first_compare)//第一个比较点
                                                  {
  if (motor_id >= motor_count || toggle_table == nullptr || toggle_count < 2U) {
    return HAL_ERROR;
  }

  auto &cfg = m_cfg[motor_id];
  auto &st = m_state[motor_id];
  auto *htim = m_timer.get_htim();
  auto *hdma = cfg.dma->get_handle();

  if (htim == nullptr || hdma == nullptr) {
    return HAL_ERROR;
  }

  if (st.busy) {
    return HAL_BUSY;
  }

  // 先停止其他的
  (void)stop_motor(motor_id);
 //先写方向
  HAL_GPIO_WritePin(cfg.dir_port, cfg.dir_pin, dir_level);

  // 先手动装第一个比较点
  m_oc.set_compare(cfg.channel, first_compare);

  // 清这个通道的 CC 标志
  switch (cfg.channel) {
  case TIM_CHANNEL_1:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
    break;
  case TIM_CHANNEL_2:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
    break;
  case TIM_CHANNEL_3:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
    break;
  case TIM_CHANNEL_4:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);
    break;
  default:
    return HAL_ERROR;
  }

  st.toggle_table = toggle_table;
  st.toggle_count = toggle_count;
  st.busy = true;
  st.wait_last_cc = false;
  st.dma_error = false;

  // 从第 2 个翻转开始交给 DMA
  if (toggle_count > 1U) {
    HAL_StatusTypeDef ret = start_dma_tail(motor_id);
    if (ret != HAL_OK) {
      (void)stop_motor(motor_id);
      return ret;
    }
  }

  // DMA 传输期间先不开 CC 中断，避免中间每次 compare 都进中断
  __HAL_TIM_DISABLE_IT(htim, channel_to_it(cfg.channel));

  HAL_StatusTypeDef ret = m_oc.oc_start(cfg.channel);
  if (ret != HAL_OK) {
    (void)stop_motor(motor_id);
    return ret;
  }

#if defined(TIM_BDTR_MOE)
  if (IS_TIM_BREAK_INSTANCE(htim->Instance)) {
    __HAL_TIM_MOE_ENABLE(htim);
  }
#endif

  return HAL_OK;
}
//一个后面的翻转交给这个函数
HAL_StatusTypeDef multi_stepper_dma::start_dma_tail(std::size_t motor_id) {
  auto &cfg = m_cfg[motor_id];
  auto &st = m_state[motor_id];
  auto *htim = m_timer.get_htim();

  if (htim == nullptr || cfg.dma == nullptr || st.toggle_count < 2U) {
    return HAL_ERROR;
  }

  volatile uint32_t *ccr = channel_to_ccr(htim, cfg.channel);
  if (ccr == nullptr) {
    return HAL_ERROR;
  }

  // dma_proxy.start() 会自己绑定 Parent 和完成/错误回调。
  cfg.dma->start(&st.toggle_table[1], ccr, st.toggle_count - 1U);
  //开启DMA
  __HAL_TIM_ENABLE_DMA(htim, channel_to_dma_req(cfg.channel));
  return HAL_OK;
}

HAL_StatusTypeDef multi_stepper_dma::stop_motor(std::size_t motor_id) {
  if (motor_id >= motor_count) {
    return HAL_ERROR;
  }

  auto &cfg = m_cfg[motor_id];
  auto &st = m_state[motor_id];
  auto *htim = m_timer.get_htim();
  auto *hdma = cfg.dma ? cfg.dma->get_handle() : nullptr;

  if (htim == nullptr) {
    return HAL_ERROR;
  }

  __HAL_TIM_DISABLE_DMA(htim, channel_to_dma_req(cfg.channel));
  __HAL_TIM_DISABLE_IT(htim, channel_to_it(cfg.channel));

  if (hdma != nullptr) {
    (void)HAL_DMA_Abort(hdma);
  }

  (void)m_oc.oc_stop(cfg.channel);

  switch (cfg.channel) {
  case TIM_CHANNEL_1:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
    break;
  case TIM_CHANNEL_2:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
    break;
  case TIM_CHANNEL_3:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
    break;
  case TIM_CHANNEL_4:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);
    break;
  default:
    break;
  }

  st = {};
  return HAL_OK;
}

HAL_StatusTypeDef multi_stepper_dma::emergency_stop_all() {
  HAL_StatusTypeDef ret = HAL_OK;
  for (std::size_t i = 0; i < motor_count; ++i) {
    if (stop_motor(i) != HAL_OK) {
      ret = HAL_ERROR;
    }
  }
  return ret;
}
//DMA完成回调
void multi_stepper_dma::on_dma_complete(std::size_t motor_id) {
  if (motor_id >= motor_count) {
    return;
  }

  auto &cfg = m_cfg[motor_id];
  auto &st = m_state[motor_id];
  auto *htim = m_timer.get_htim();

  if (htim == nullptr || !st.busy) {
    return;
  }

  // 最后一个 compare 已经写进 CCRx，但最后一次翻转还没真正发生。
  __HAL_TIM_DISABLE_DMA(htim, channel_to_dma_req(cfg.channel));
  st.wait_last_cc = true;

  // 先清标志，再开这个通道自己的 CC 中断，等最后一次 compare 到来后 stop
  switch (cfg.channel) {
  case TIM_CHANNEL_1:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1);
    break;
  case TIM_CHANNEL_2:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC2);
    break;
  case TIM_CHANNEL_3:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
    break;
  case TIM_CHANNEL_4:
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC4);
    break;
  default:
    return;
  }

  __HAL_TIM_ENABLE_IT(htim, channel_to_it(cfg.channel));
}
//DMA错误回调
void multi_stepper_dma::on_dma_error(std::size_t motor_id) {
  if (motor_id >= motor_count) {
    return;
  }
  m_state[motor_id].dma_error = true;
  (void)stop_motor(motor_id);
}
//CC事件回调
void multi_stepper_dma::on_cc_event(std::size_t motor_id) {
  if (motor_id >= motor_count) {
    return;
  }

  auto &st = m_state[motor_id];
  if (!st.busy || !st.wait_last_cc) {
    return;
  }

  st.wait_last_cc = false;
  (void)stop_motor(motor_id);
}
//通道到DMA请求
uint32_t multi_stepper_dma::channel_to_dma_req(uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return TIM_DMA_CC1;
  case TIM_CHANNEL_2:
    return TIM_DMA_CC2;
  case TIM_CHANNEL_3:
    return TIM_DMA_CC3;
  case TIM_CHANNEL_4:
    return TIM_DMA_CC4;
  default:
    return 0U;
  }
}
//通道到中断
uint32_t multi_stepper_dma::channel_to_it(uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return TIM_IT_CC1;
  case TIM_CHANNEL_2:
    return TIM_IT_CC2;
  case TIM_CHANNEL_3:
    return TIM_IT_CC3;
  case TIM_CHANNEL_4:
    return TIM_IT_CC4;
  default:
    return 0U;
  }
}
//通道到CCR
volatile uint32_t *multi_stepper_dma::channel_to_ccr(TIM_HandleTypeDef *htim,
                                                     uint32_t channel) {
  if (htim == nullptr) {
    return nullptr;
  }

  switch (channel) {
  case TIM_CHANNEL_1:
    return &htim->Instance->CCR1;
  case TIM_CHANNEL_2:
    return &htim->Instance->CCR2;
  case TIM_CHANNEL_3:
    return &htim->Instance->CCR3;
  case TIM_CHANNEL_4:
    return &htim->Instance->CCR4;
  default:
    return nullptr;
  }
}
//通道到索引
std::size_t multi_stepper_dma::channel_to_index(uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return 0U;
  case TIM_CHANNEL_2:
    return 1U;
  case TIM_CHANNEL_3:
    return 2U;
  case TIM_CHANNEL_4:
    return 3U;
  default:
    return motor_count;
  }
}

} // namespace app
