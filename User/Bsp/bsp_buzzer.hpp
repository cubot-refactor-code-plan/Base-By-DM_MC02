/**
 * @file bsp_buzzer.hpp
 * @author Rh
 * @brief 蜂鸣器驱动 —— PWM 无源蜂鸣器（默认 TIM12 CH2 PB15）
 * @version 0.3
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details 使用定时器 PWM 驱动无源蜂鸣器，定时器/通道/基频/提示音参数
 *          全部通过 Config 传入（默认 TIM12 CH2）。
 *          TIM12: PSC=23, ARR 动态调整以改变频率。
 *          计数基频 base_clk = 定时器主频 / (PSC + 1)，TIM12 ≈ 6 MHz。
 *
 * @note 初始化示例（默认配置）：
 *
 *       // 参数顺序：htim, channel, base_clk, default_freq, freq_min, freq_max, short_ms, long_ms, gap_ms
 *       BspBuzzer buzzer2({&htim3, TIM_CHANNEL_1, 1000000UL}); // 1 MHz 基频，其余默认
 *
 * @note 使用示例：
 *
 *       bsp_buzzer.tone(3000, 50);  // 3kHz, 50% 占空比
 *       vTaskDelay(pdMS_TO_TICKS(500));
 *       bsp_buzzer.off();           // 关闭
 *
 *       bsp_buzzer.beep(2000, 100); // 2kHz 响 100ms 后自动关闭
 *       bsp_buzzer.beep_pass_short(); // 短鸣（通过提示）
 */

#ifndef __BSP_BUZZER_HPP__
#define __BSP_BUZZER_HPP__

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"
#include "tim.h" // IWYU pragma: keep


/**
 * @brief 蜂鸣器驱动类
 *
 * @note 封装定时器 PWM，提供 tone/off/beep 等接口。
 *       无 RTOS 资源（无队列、无定时器），纯硬件 PWM 控制。
 */
class BspBuzzer
{
public:
  /**
   * @brief 蜂鸣器配置结构体（全部带默认值：TIM12 CH2 PB15）
   *
   * @note 换用其他定时器时只需改 htim/channel/base_clk 三项。
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序，可匿名传入）
     */
    Config(TIM_HandleTypeDef *htim = &htim12, uint32_t channel = TIM_CHANNEL_2, uint32_t base_clk = 6000000UL, uint32_t default_freq = 3000UL, uint32_t freq_min = 50, uint32_t freq_max = 20000, uint32_t short_ms = 80, uint32_t long_ms = 400, uint32_t gap_ms = 100)

      : htim(htim),
        channel(channel),
        base_clk(base_clk),
        default_freq(default_freq),
        freq_min(freq_min),
        freq_max(freq_max),
        short_ms(short_ms),
        long_ms(long_ms),
        gap_ms(gap_ms)
    {
    }

    TIM_HandleTypeDef *htim;         ///< PWM 定时器句柄
    uint32_t           channel;      ///< PWM 通道
    uint32_t           base_clk;     ///< 计数基频 (Hz) = 定时器主频 / (PSC+1)
    uint32_t           default_freq; ///< 默认鸣叫频率 (Hz)
    uint32_t           freq_min;     ///< 频率下限 (Hz)
    uint32_t           freq_max;     ///< 频率上限 (Hz)
    uint32_t           short_ms;     ///< 短鸣时长 (ms)
    uint32_t           long_ms;      ///< 长鸣时长 (ms)
    uint32_t           gap_ms;       ///< 鸣叫间隔 (ms)
  };

  BspBuzzer()  = default;
  ~BspBuzzer() = default;

  /**
   * @brief 用配置结构体构造
   * @param cfg 定时器/通道/音长等配置
   */
  BspBuzzer(const Config &cfg);

  /**
   * @brief 绑定配置（默认构造后调用，可重复调用换配置）
   * @param cfg 定时器/通道/音长等配置
   */
  void init(const Config &cfg);

  /**
   * @brief 以指定频率和音量启动 PWM
   * @param freq_hz    频率 (Hz)，自动限幅到 Config::freq_min ~ freq_max
   * @param volume_pct 占空比 (%)，0~100
   */
  void tone(uint32_t freq_hz, uint32_t volume_pct = 50);

  ///< 停止 PWM
  void off();

  /**
   * @brief 鸣叫指定时长后自动关闭（阻塞）
   * @param freq_hz     频率 (Hz)
   * @param duration_ms 时长 (ms)
   */
  void beep(uint32_t freq_hz, uint32_t duration_ms);

  ///< 短鸣（默认频率，通过提示）
  void beep_pass_short();

  ///< 长鸣（默认频率，开始提示）
  void beep_start_long();

  ///< 三短鸣（失败提示）
  void beep_fail();

private:
  ///< 设置 ARR 和 CCR（内部辅助）
  void set_arr_and_ccr(uint32_t arr, uint32_t volume_pct);

  Config _config; ///< 蜂鸣器配置（定时器/通道/基频/音长参数）
};

#endif // __BSP_BUZZER_HPP__
