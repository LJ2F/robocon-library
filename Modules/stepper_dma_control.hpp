#ifndef MULTI_STEPPER_DMA_HPP
#define MULTI_STEPPER_DMA_HPP

#include "bsp_dma.hpp"
#include "bsp_timer.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace gdut {

class multi_stepper_dma {
public:
  struct motor_config {
    uint32_t channel{0};             // TIM_CHANNEL_1~4
    gdut::dma_proxy *dma{nullptr};   // 该通道对应的 DMA
    GPIO_TypeDef *dir_port{nullptr}; // 方向脚
    uint16_t dir_pin{0};
    uint16_t pulse_high_ticks{0}; // STEP 高电平宽度
  };

  struct motor_state {
    const uint32_t *toggle_table{nullptr}; // [rise0, fall0, rise1, fall1, ...]
    std::size_t toggle_count{0};           // 总翻转次数 = 2 * pulse_count
    bool busy{false};                      // 是否正在运动
    bool wait_last_cc{false};              // 是否正在等待最后一个比较事件
    bool dma_error{false};                 // 是否 DMA 错误
  };

  static constexpr std::size_t motor_count =
      4; // 一个 TIM 最多支持 4 路输出比较

  multi_stepper_dma(gdut::timer &timer,
                    const std::array<motor_config, motor_count> &configs);

  HAL_StatusTypeDef init();

  HAL_StatusTypeDef start_motor(std::size_t motor_id,
                                const uint32_t *toggle_table,
                                std::size_t toggle_count,
                                GPIO_PinState dir_level,
                                uint32_t first_compare);

  HAL_StatusTypeDef stop_motor(std::size_t motor_id);
  HAL_StatusTypeDef emergency_stop_all();

  bool is_busy(std::size_t motor_id) const;
  bool has_dma_error(std::size_t motor_id) const;

  // 由 HAL_TIM_OC_DelayElapsedCallback -> timer.call_capture_callback(...)
  // 转发到这里，处理每个通道自己的比较事件。
  void on_cc_event(std::size_t motor_id);

  // period_table[i] = 第 i 个脉冲的总周期
  // out_toggle_table = [rise0, fall0, rise1, fall1, ...]
  static std::size_t
  build_toggle_table(const uint16_t *period_table, std::size_t pulse_count,
                     uint16_t pulse_high_ticks, uint32_t first_rise_tick,
                     uint32_t *out_toggle_table, std::size_t out_cap);

private:
  HAL_StatusTypeDef start_dma_tail(std::size_t motor_id);

  void on_dma_complete(std::size_t motor_id);
  void on_dma_error(std::size_t motor_id);

  static uint32_t channel_to_dma_req(uint32_t channel);
  static uint32_t channel_to_it(uint32_t channel);
  static volatile uint32_t *channel_to_ccr(TIM_HandleTypeDef *htim,
                                           uint32_t channel);
  static std::size_t channel_to_index(uint32_t channel);

private:
  gdut::timer &m_timer;
  gdut::timer::timer_proxy m_timer_proxy;
  gdut::timer::timer_oc m_oc;
  std::array<motor_config, motor_count> m_cfg;
  std::array<motor_state, motor_count> m_state;
};

} // namespace gdut

#endif // MULTI_STEPPER_DMA_HPP
