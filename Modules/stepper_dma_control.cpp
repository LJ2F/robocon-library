#include "steppr_dma_control.hpp"

#include <algorithm>

namespace app {

stepper_dma_controller::stepper_dma_controller(gdut::timer &step_timer,
                                               gdut::dma_proxy &dma,
                                               const config &cfg)
    : m_timer(step_timer), m_dma(dma), m_timer_proxy(&step_timer),
      m_pwm(&step_timer), m_cfg(cfg) {}

HAL_StatusTypeDef stepper_dma_controller::init() {
  if (m_timer.get_htim() == nullptr || m_dma.get_handle() == nullptr ||
      m_cfg.dir_port == nullptr || m_cfg.step_high_ticks == 0U) {
    return HAL_ERROR;
  }

  // 这里默认 DMA 的 stream/channel/request 已经在 CubeMX 或 MSP 里配好。
  // 只借用 dma_proxy 做 init 和句柄管理。
  m_dma.init();

  // 运行时让 timer 接管这个 DMA 句柄的完成/错误回调，防止两个冲突，兔同时处理。
  m_timer.attach_dma(m_dma.get_handle());
  // 注册回调函数
  register_callbacks();

  return HAL_OK;
}
//采用timer 的dma回调函数，完成/错误/周期回调
void stepper_dma_controller::register_callbacks() {
  m_timer.register_dma_xfer_cplt_callback([this]() { on_dma_complete(); });
  m_timer.register_dma_error_callback([this]() { on_dma_error(); });
  m_timer.register_period_elapsed_callback([this]() { on_period_elapsed(); });
}

HAL_StatusTypeDef stepper_dma_controller::start_move(const uint16_t *arr_table,
                                                     std::size_t pulse_count,
                                                     GPIO_PinState dir_level) {
  if (arr_table == nullptr || pulse_count == 0U || m_busy) {
    return HAL_ERROR;
  }

  TIM_HandleTypeDef *htim = m_timer.get_htim();
  DMA_HandleTypeDef *hdma = m_dma.get_handle();
  if (htim == nullptr || hdma == nullptr) {
    return HAL_ERROR;
  }
//必须判断步高时间不能小于1个定时器周期
  if (arr_table[0] <= m_cfg.step_high_ticks) {
    return HAL_ERROR;
  }
//参数的传递
  m_arr_table = arr_table;
  m_pulse_count = pulse_count;
  m_busy = true;
  m_wait_final_period = false;
  m_dma_error = false;
//设置方向
  HAL_GPIO_WritePin(m_cfg.dir_port, m_cfg.dir_pin, dir_level);

  // 清掉旧状态。
  (void)HAL_TIM_PWM_Stop(htim, m_cfg.pwm_channel);
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
  __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
  (void)HAL_DMA_Abort(hdma);
  __HAL_TIM_SET_COUNTER(htim, 0U);

  prepare_first_period(arr_table[0]);

  if (pulse_count == 1U) {
    // 只有一个脉冲时，不开 DMA。
    m_wait_final_period = true;
    m_timer.enable_it(TIM_IT_UPDATE);

    HAL_StatusTypeDef ret = m_pwm.pwm_start(m_cfg.pwm_channel);
    if (ret != HAL_OK) {
      (void)stop();
    }
    return ret;
  }

  HAL_StatusTypeDef ret = start_dma_tail();
  if (ret != HAL_OK) {
    (void)stop();
    return ret;
  }

  // DMA 传输期间先不开更新中断，避免与最后一次的逻辑冲突
  __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);

  ret = m_pwm.pwm_start(m_cfg.pwm_channel);
  if (ret != HAL_OK) {
    (void)stop();
  }
  return ret;
}

void stepper_dma_controller::prepare_first_period(uint16_t first_arr) {
  TIM_HandleTypeDef *htim = m_timer.get_htim();
  if (htim == nullptr) {
    return;
  }

  m_pwm.set_duty(m_cfg.pwm_channel, m_cfg.step_high_ticks);
  m_timer_proxy.set_arr(first_arr);
  m_timer_proxy.set_counter(0U);

  // 把预装载值立刻刷进活动寄存器，保证第一周期就按 table[0] 运行。
  (void)HAL_TIM_GenerateEvent(htim, TIM_EVENTSOURCE_UPDATE);
}
//第 2 个脉冲开始到最后一个脉冲”的 ARR 数据交给 DMA，
// 让 DMA 在定时器更新事件时自动把新的周期值写进定时器的 ARR 寄存器。
HAL_StatusTypeDef stepper_dma_controller::start_dma_tail() {
  TIM_HandleTypeDef *htim = m_timer.get_htim();
  DMA_HandleTypeDef *hdma = m_dma.get_handle();
  if (htim == nullptr || hdma == nullptr || m_pulse_count < 2U) {
    return HAL_ERROR;
  }

  // dma_proxy::start() 会重新改 Parent/回调，和 timer::attach_dma() 冲突。
  // 所以这里直接调用 HAL_DMA_Start_IT
  HAL_StatusTypeDef ret = HAL_DMA_Start_IT(
      hdma, reinterpret_cast<uint32_t>(&m_arr_table[1]),
      reinterpret_cast<uint32_t>(&htim->Instance->ARR), m_pulse_count - 1U);
  if (ret != HAL_OK) {
    return ret;
  }

  __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_UPDATE);
  return HAL_OK;
}

HAL_StatusTypeDef stepper_dma_controller::stop() {
  TIM_HandleTypeDef *htim = m_timer.get_htim();
  DMA_HandleTypeDef *hdma = m_dma.get_handle();
  if (htim == nullptr || hdma == nullptr) {
    return HAL_ERROR;
  }

  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
  __HAL_TIM_DISABLE_IT(htim, TIM_IT_UPDATE);
  (void)HAL_DMA_Abort(hdma);
  (void)m_pwm.pwm_stop(m_cfg.pwm_channel);

  m_arr_table = nullptr;
  m_pulse_count = 0U;
  m_busy = false;
  m_wait_final_period = false;
  return HAL_OK;
}

HAL_StatusTypeDef stepper_dma_controller::emergency_stop() {
  return stop();
}

void stepper_dma_controller::on_dma_complete() {
  TIM_HandleTypeDef *htim = m_timer.get_htim();
  if (htim == nullptr || !m_busy) {
    return;
  }

  // 现在最后一个 ARR 已经写进寄存器了，但最后一个 PWM 周期还没跑完。
  // 所以：
  // 1) 先关掉 DMA 请求，防止继续请求 DMA
  // 2) 打开更新中断
  // 3) 等下一个更新事件到来，再真正 stop()
  __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_UPDATE);
  m_wait_final_period = true;
  m_timer.enable_it(TIM_IT_UPDATE);
}

void stepper_dma_controller::on_dma_error() {
  m_dma_error = true;
  (void)stop();
}

void stepper_dma_controller::on_period_elapsed() {
  if (!m_busy || !m_wait_final_period) {
    return;
  }

  m_wait_final_period = false;
  (void)stop();
}
//构建梯形运动数组生成trapezoid_arr_table
//buf: 输出数组，用于存储梯形运动数组，每个元素为 uint16_t 类型
//arr_start: 初始 ARR 值，必须大于等于 arr_min
//arr_min: 最小 ARR 值，必须小于等于 arr_start
//accel_steps: 加速步数，必须大于等于 0
//decel_steps: 减速步数，必须大于等于 0
//返回值: 实际生成的梯形运动数组长度，可能小于 max_len
std::size_t stepper_dma_controller::build_trapezoid_arr_table(
    uint16_t *buf, std::size_t max_len, std::size_t steps, uint16_t arr_start,
    uint16_t arr_min, std::size_t accel_steps, std::size_t decel_steps) {
  if (buf == nullptr || max_len == 0U || steps == 0U || arr_start < arr_min) {
    return 0U;
  }

  steps = std::min(steps, max_len);
  if (accel_steps + decel_steps > steps) {
    accel_steps = steps / 2U;
    decel_steps = steps - accel_steps;
  }

  const std::size_t const_steps = steps - accel_steps - decel_steps;
  const uint32_t delta = static_cast<uint32_t>(arr_start - arr_min);
  std::size_t idx = 0U;

  for (std::size_t i = 0U; i < accel_steps; ++i) {
    const uint32_t div = (accel_steps == 0U) ? 1U : static_cast<uint32_t>(accel_steps);
    uint16_t arr = static_cast<uint16_t>(arr_start - (delta * i) / div);
    if (arr < arr_min) {
      arr = arr_min;
    }
    buf[idx++] = arr;
  }

  for (std::size_t i = 0U; i < const_steps; ++i) {
    buf[idx++] = arr_min;
  }

  for (std::size_t i = 0U; i < decel_steps; ++i) {
    const uint32_t div = (decel_steps == 0U) ? 1U : static_cast<uint32_t>(decel_steps);
    uint16_t arr = static_cast<uint16_t>(arr_min + (delta * i) / div);
    if (arr > arr_start) {
      arr = arr_start;
    }
    buf[idx++] = arr;
  }

  return idx;
}

} // namespace app
