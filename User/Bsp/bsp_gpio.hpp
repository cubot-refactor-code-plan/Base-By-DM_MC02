/**
 * @file bsp_gpio.hpp
 * @author Rh
 * @brief GPIO 统一封装 — 输出引脚控制
 * @version 0.4
 * @date 2026-09-10
 *
 * @copyright Copyright (c) 2026
 *
 * @details BspGpio 类：GPIO 输出引脚的 set/reset/toggle/write/read 封装。
 *          引脚方向/上下拉由 CubeMX 的 MX_GPIO_Init() 配置，本类仅封装电平控制。
 *
 * @note 初始化示例：
 *
 *   // 1. 构造时绑定（推荐）
 *   BspGpio power({GPIOA, GPIO_PIN_5});
 *
 *   // 2. 默认构造 + init()（可重复绑定换引脚）
 *   BspGpio led;
 *   led.init({GPIOA, GPIO_PIN_5});
 *
 * @note 使用示例：
 *
 *   led.set();            // 高电平
 *   led.reset();          // 低电平
 *   led.toggle();         // 翻转
 *   led.write(false);     // 输出低电平
 *   bool s = led.read();  // 读取引脚电平
 *
 */

#ifndef __BSP_GPIO_HPP__
#define __BSP_GPIO_HPP__

#include "main.h" // IWYU pragma: keep
#include <stdint.h>

#include "status.hpp" // 统一状态码


/* ==================== GPIO 输出引脚封装类 ==================== */

/**
 * @brief GPIO 输出引脚封装类
 *
 * @note 硬件引脚方向/上下拉由 CubeMX 的 MX_GPIO_Init() 配置，
 *       本类仅保存端口/引脚并封装电平操作。
 */
class BspGpio
{
public:
  /**
   * @brief GPIO 引脚配置结构体（可匿名按序传入）
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     * @param port GPIO 端口（GPIOA / GPIOB / ...）
     * @param pin  引脚掩码（GPIO_PIN_x）
     */
    Config(GPIO_TypeDef *port = nullptr, uint16_t pin = 0U)
      : port(port),
        pin(pin)
    {
    }

    GPIO_TypeDef *port; ///< GPIO 端口 (GPIOA / GPIOB / ...)
    uint16_t      pin;  ///< 引脚掩码 (GPIO_PIN_x)
  };

  BspGpio() = default;

  /**
   * @brief 构造函数（只做赋值，硬件已由 CubeMX 初始化）
   * @param cfg 端口 + 引脚（可匿名按序传入）
   */
  BspGpio(const Config &cfg) : _port(cfg.port), _pin(cfg.pin)
  {
  }

  /**
   * @brief 绑定引脚（可重复调用换引脚）
   * @param cfg 端口 + 引脚
   * @return Status OK=绑定成功，BAD_ARG=端口/引脚非法
   */
  Status init(const Config &cfg)
  {
    if (cfg.port == nullptr || cfg.pin == 0U)
    {
      return Status::BAD_ARG; // 端口/引脚非法
    }
    _port = cfg.port;
    _pin  = cfg.pin;
    return Status::OK;
  }

  ///< 输出高电平
  void set() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_SET); }

  ///< 输出低电平
  void reset() const { HAL_GPIO_WritePin(_port, _pin, GPIO_PIN_RESET); }

  ///< 翻转电平
  void toggle() const { HAL_GPIO_TogglePin(_port, _pin); }

  ///< 输出指定电平
  void write(bool state) const
  {
    HAL_GPIO_WritePin(_port, _pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  ///< 读取当前引脚电平
  bool read() const { return (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET); }

  ///< 获取绑定的 GPIO 端口
  GPIO_TypeDef *get_port() const { return _port; }

  ///< 获取绑定的引脚掩码
  uint16_t get_pin() const { return _pin; }

private:
  GPIO_TypeDef *_port = nullptr; ///< GPIO 端口指针
  uint16_t      _pin  = 0U;      ///< 引脚掩码
};

#endif // __BSP_GPIO_HPP__
