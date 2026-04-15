#ifndef STEPPER_DMA_CONTROLLER_HPP
#define STEPPER_DMA_CONTROLLER_HPP

#include "bsp_dma.hpp"
#include "bsp_timer.hpp"

#include <cstddef>
#include <cstdint>

namespace app {

class stepper_dma_controller {
public:
  struct config {
    uint32_t pwm_channel{TIM_CHANNEL_1};//pwm通道的绑定
    GPIO_TypeDef *dir_port{nullptr};  //方向引脚的绑定
    uint16_t dir_pin{0};  //方向引脚的绑定
    uint32_t step_high_ticks{20U};
  };
//绑定timer和dma
  stepper_dma_controller(gdut::timer &step_timer, gdut::dma_proxy &dma,
                         const config &cfg);

  HAL_StatusTypeDef init();
  //启动运动，arr_table是步长表，pulse_count是步长表的长度，dir_level是方向引脚的状态，
  HAL_StatusTypeDef start_move(const uint16_t *arr_table,
                               std::size_t pulse_count,
                               GPIO_PinState dir_level);
//停止运动
  HAL_StatusTypeDef stop();
  HAL_StatusTypeDef emergency_stop();
//判断是否正在运动
  [[nodiscard]] bool busy() const { return m_busy; }
//判断是否正在等待最后一个周期，因为最后一个发送后就等待最后一个周期了，不能直接暂停
  [[nodiscard]] bool wait_final_period() const { return m_wait_final_period; }
  [[nodiscard]] std::size_t pulse_count() const { return m_pulse_count; }
//建立梯形运动的步长表，buf是步长表的缓冲区，max_len是步长表的最大长度，steps是步长表的步长
//arr_start是步长表的起始值，arr_min是步长表的最小值，accel_steps是加速步长，decel_steps是减速步长
//返回值是步长表的长度
//采用的方法是先计算加速步长，然后计算减速步长，然后计算中间步长，中间步长是加速步长和减速步长之和
  static std::size_t build_trapezoid_arr_table(uint16_t *buf,
                                               std::size_t max_len,
                                               std::size_t steps,
                                               uint16_t arr_start,
                                               uint16_t arr_min,
                                               std::size_t accel_steps,
                                               std::size_t decel_steps);
//别误会，视觉分组
private:
  void register_callbacks();//注册回调函数
  void prepare_first_period(uint16_t first_arr);//准备第一个周期，DMA的要第一个启动
  HAL_StatusTypeDef start_dma_tail();//启动DMA后续的函数
  void on_dma_complete();//DMA完成回调函数
  void on_dma_error();//DMA错误回调函数
  void on_period_elapsed();//周期回调函数

private:
  gdut::timer &m_timer;
  gdut::dma_proxy &m_dma;
  gdut::timer::timer_proxy m_timer_proxy;
  gdut::timer::timer_pwm m_pwm;
  config m_cfg{};

  const uint16_t *m_arr_table{nullptr};
  std::size_t m_pulse_count{0};

  volatile bool m_busy{false};//是否正在运动
  volatile bool m_wait_final_period{false};//是否正在等待最后一个周期
  volatile bool m_dma_error{false};//是否DMA错误
};

} // namespace app

#endif // STEPPER_DMA_CONTROLLER_HPP
