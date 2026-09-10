#include "bsp_key.hpp"

/* ==================== 初始化 ==================== */

/**
 * @brief 绑定引脚并重置消抖状态
 * @param cfg 引脚 + 消抖/长按参数
 *
 * @note 读取 GPIO 初始电平作为 _last_stable，
 *       避免启动时误触发按下/释放事件。
 *
 * @return Status OK=绑定成功，BAD_ARG=端口/引脚非法
 */
Status BspKey::init(const Config &cfg)
{
  if (cfg.port == nullptr || cfg.pin == 0U)
  {
    return Status::BAD_ARG; // 端口/引脚非法
  }

  _port           = cfg.port;
  _pin            = cfg.pin;
  _active_low     = cfg.active_low;
  _debounce_cnt   = cfg.debounce_cnt;
  _long_press_cnt = cfg.long_press_cnt;
  _cnt            = 0U;
  _hold_cnt       = 0U;
  _long_fired     = false;

  /* 以当前电平作为初始稳定状态 */
  bool raw     = (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
  _last_stable = _active_low ? !raw : raw;

  return Status::OK;
}

/* ==================== 消抖轮询 ==================== */

/**
 * @brief 单次消抖轮询（裸机非阻塞，立即返回）
 *
 * 状态转换图:
 * @code
 *   IDLE ──(cnt>=N, active)──► PRESSED  ──(hold++)──► ... ──► LONG_PRESS
 *   PRESSED ──(cnt>=N, !active)──► RELEASED ──► IDLE
 *   LONG_PRESS 后的释放 ──► NONE（不产生 RELEASED，避免误判为短按）
 * @endcode
 *
 * @return 本次触发的事件，大部分情况返回 NONE
 */
BspKey::Event BspKey::poll()
{
  bool raw    = (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
  bool active = _active_low ? !raw : raw; /* true = 按键按下 */

  /* ---- 阶段一: 电平与上次一致 → 稳定 ---- */
  if (active == _last_stable)
  {
    _cnt = 0U; /* 清零抖动计数 */
    if (active)
    {
      _hold_cnt++; /* 按下保持中 */
      if (!_long_fired && _hold_cnt >= _long_press_cnt)
      {
        _long_fired = true; /* 标记已触发，本次按下不再重复 */
        return Event::LONG_PRESS;
      }
    }
    return Event::NONE;
  }

  /* ---- 阶段二: 电平与上次不同 → 可能是抖动 ---- */
  _cnt++;
  if (_cnt < _debounce_cnt)
  {
    return Event::NONE; /* 消抖未完成，继续等待 */
  }

  /* ---- 阶段三: 消抖确认，状态切换 ---- */
  _cnt = 0U;

  if (active && !_last_stable)
  {
    /* 确认按下 */
    _last_stable = true;
    _hold_cnt    = 0U;
    _long_fired  = false;
    return Event::PRESSED;
  }
  else if (!active && _last_stable)
  {
    /* 确认释放 */
    _last_stable = false;
    _hold_cnt    = 0U;
    if (_long_fired)
    {
      /* 长按已触发过：本次释放不产生 RELEASED（避免被上层误判为一次短按） */
      _long_fired = false;
      return Event::NONE;
    }
    return Event::RELEASED;
  }

  return Event::NONE;
}

/* ==================== 辅助接口 ==================== */

/**
 * @brief 读取当前引脚原始电平
 */
bool BspKey::read() const
{
  return (HAL_GPIO_ReadPin(_port, _pin) != GPIO_PIN_RESET);
}
