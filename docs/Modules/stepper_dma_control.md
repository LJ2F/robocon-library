### 你至少要准备：&#x20;

* 一个定时器，比如 TIM8

* 这个定时器的若干输出比较通道，比如 CH1~CH4

* 每个通道对应一个 DMA

* 每个电机一个方向 GPIO

* 一个连续有效的翻转时间表 toggle_table

* 一个脉冲周期表 period_table

***

## 3. CubeMX 应该怎么配

### 定时器

建议这样理解：

* 定时器基准频率要足够高

* 让计数单位尽量细一些，例如 1us 一个 tick，或者更快

* 通道模式用 Output Compare Toggle

* 使能对应通道的 DMA 请求

* 使能对应 DMA 中断

* 使能定时器中断

### DMA

每路电机一个 DMA，方向大概应当满足：

* 内存到外设

* memory increment 开

* peripheral increment 关

* 数据宽度和 CCR 寄存器匹配

* normal 模式

### GPIO

* DIR 普通输出

* STEP 对应定时器复用引脚

* ![](%E6%AD%A5%E8%BF%9B%E7%94%B5%E6%9C%BA%E7%9A%84%E9%85%8D%E7%BD%AE_md_files/6ec73690-3c47-11f1-9ecd-05b583adcd41.jpeg?v=1&type=image)

* ![](%E6%AD%A5%E8%BF%9B%E7%94%B5%E6%9C%BA%E7%9A%84%E9%85%8D%E7%BD%AE_md_files/9c7405f0-3c47-11f1-9ecd-05b583adcd41.jpeg?v=1&type=image)

* ![](%E6%AD%A5%E8%BF%9B%E7%94%B5%E6%9C%BA%E7%9A%84%E9%85%8D%E7%BD%AE_md_files/a431d5b0-3c47-11f1-9ecd-05b583adcd41.jpeg?v=1&type=image)

***

## 4. 定时器 tick 怎么理解

你的 period_table 和 pulse_high_ticks 这些参数，单位都不是秒，也不是毫秒。

它们的单位是：

定时器 tick

比如：

* 定时器时钟分到 1 MHz

* 那么 1 tick = 1 us

这时：

* pulse_high_ticks = 20 就表示高电平 20 us

* period_table[i] = 200 就表示该步周期 200 us

所以实际频率大约是：

1 / 200us = 5kHz

***

## 5. pulse_high_ticks 该怎么设

这个值必须满足：

小于每一个 period_table[i]

因为代码里明确检查了：

```
if (period <= pulse_high_ticks) return 0;
```

也就是说，一个脉冲总周期必须比高电平宽度长，不然高电平还没结束，下一个脉冲就来了。

对小白来说，先用一个保守值：

* 如果 tick = 1us

* 先设 pulse_high_ticks = 10~20

通常足够大多数 STEP/DIR 驱动器识别。

***

## 6. first_rise_tick 该怎么设

这个值表示：

第一个上升沿在计数器跑到多少时发生

因为你的定时器在 init() 之后就一直跑着，所以这个值必须是“未来的某个时刻”，不能已经过去。

最稳妥的思路是：

```
first_rise_tick = 当前CNT + 安全提前量
```

比如当前 CNT = 1000，你可以让它从 1100 开始。
这样 DMA、OC 都来得及准备。

***

## 7. 一次完整使用流程

你按下面顺序做就行：

### 第一步：建对象

* 建 gdut::timer

* 建每路 gdut::dma_proxy

* 准备 motor_config 数组

* 建 multi_stepper_dma

### 第二步：调用 init()

这一步会：

* 注册回调

* 初始化 DMA

* 启动 timer

### 第三步：准备 period_table

比如你想发 200 个脉冲，可以准备一个长度为 200 的数组。
如果要匀速，就全部填一样的值。
如果要加减速，就前面大一点、中间小一点、后面再变大。

### 第四步：准备 toggle_table

调用 build_toggle_table() 把 period_table 转成翻转表。

### 第五步：启动电机

调用：

```
start_motor(motor_id, toggle_table, toggle_count, dir_level, toggle_table[0]);
```

其中最后一个参数通常就填 toggle_table[0]。

### 第六步：等待完成

可以轮询：

* is_busy(motor_id)

* has_dma_error(motor_id)

### 第七步：必要时停止

* 正常停：等它自己跑完

* 急停：stop_motor() 或 emergency_stop_all()

# 一个最容易理解的最小使用例

```markup
// 1. 先准备底层句柄（这些通常是 CubeMX 生成的）
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch1;
extern DMA_HandleTypeDef hdma_tim8_ch2;
```

```markup
// 2. 包装成你的 C++ 对象
gdut::timer g_timer(&htim8);
gdut::dma_proxy g_dma1(&hdma_tim8_ch1);
gdut::dma_proxy g_dma2(&hdma_tim8_ch2);
```

```markup
// 3. 准备每个电机的配置
// motor_config 里从 cpp 看，至少会有：
// dma, dir_port, dir_pin, pulse_high_ticks, channel
std::array<app::multi_stepper_dma::motor_config,
app::multi_stepper_dma::motor_count> cfgs;
```

```markup
// 假设 0 号电机用 CH1
cfgs[0].dma = &g_dma1;
cfgs[0].dir_port = GPIOB;
cfgs[0].dir_pin = GPIO_PIN_0;
cfgs[0].pulse_high_ticks = 20;
cfgs[0].channel = TIM_CHANNEL_1;
```

```markup
// 假设 1 号电机用 CH2
cfgs[1].dma = &g_dma2;
cfgs[1].dir_port = GPIOB;
cfgs[1].dir_pin = GPIO_PIN_1;
cfgs[1].pulse_high_ticks = 20;
cfgs[1].channel = TIM_CHANNEL_2;
```

```markup
// 其他不用的通道留空
// cfgs[2] = {};
// cfgs[3] = {};
```

```markup
app::multi_stepper_dma stepper(g_timer, cfgs);
```

```markup
// 4. 初始化
stepper.init();
```

```markup
// 5. 准备一个匀速脉冲表
constexpr std::size_t pulse_count = 200;
uint16_t period_table[pulse_count];
for (std::size_t i = 0; i < pulse_count; ++i) {
period_table[i] = 200;   // 每步周期 200 tick
}
```

```markup
// 6. 准备翻转表
uint32_t toggle_table[pulse_count * 2];
uint32_t first_rise_tick = __HAL_TIM_GET_COUNTER(&htim8) + 100;
```

```markup
std::size_t toggle_count =
stepper.build_toggle_table(period_table,
pulse_count,
20,               // 高电平 20 tick
first_rise_tick,
toggle_table,
pulse_count * 2);
```

```markup
// 7. 启动 0 号电机
if (toggle_count > 0) {
stepper.start_motor(0,
toggle_table,
toggle_count,
GPIO_PIN_SET,           // 方向
toggle_table[0]);       // 第一个比较点
}
```

```markup
```

# 你还必须补上的“中断连接”

&#x20;

这部分你这份 .cpp 没写出来，但按你 gdut::timer 和 gdut::dma_proxy 的设计，这是必须有的。这部分是我根据封装方式作出的明确推断。

你至少要保证：

## 1）DMA 中断里调用 HAL_DMA_IRQHandler

比如：

```
void DMA2_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_tim8_ch1);
}
```

## 2）TIM 中断里调用 HAL_TIM_IRQHandler

比如：

```
void TIM8_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim8);
}
```

## 3）在 HAL 的回调里，把通道事件转发给 gdut::timer

例如常见写法会类似：

```
extern gdut::timer g_timer;

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        g_timer.call_capture_callback(1);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        g_timer.call_capture_callback(2);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
        g_timer.call_capture_callback(3);
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
        g_timer.call_capture_callback(4);
    }
}
```

否则虽然你注册了 on_cc_event()，但最后一次 compare 到来时，它根本收不到通知。
