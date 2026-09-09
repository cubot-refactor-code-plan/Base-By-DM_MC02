#include "dm_motor.hpp"

#include "dji_motor.hpp"

#include "FreeRTOS.h" // IWYU pragma: keep
#include "task.h"

#include <math.h>
#include <string.h>


namespace
{
constexpr float TWO_PI = 6.28318530717958647692f;

// 对象存活期间即为进入临界保护期间
class ScopedTaskCritical
{
public:
  ScopedTaskCritical()
    : _active(xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) // 任务调度器启用时才会启用临界保护功能
  {
    if (_active)
    {
      taskENTER_CRITICAL();
    }
  }

  ~ScopedTaskCritical()
  {
    if (_active)
    {
      taskEXIT_CRITICAL();
    }
  }

private:
  bool _active;
};

bool mode_valid(DmControlMode mode) // 合法性检验
{
  return (mode == DmControlMode::MIT) ||
         (mode == DmControlMode::POSITION_VELOCITY) ||
         (mode == DmControlMode::VELOCITY) ||
         (mode == DmControlMode::POSITION_TORQUE);
}

template <DmMotorType TYPE>
struct DmMotorDefaults;

template <> // 声明这是一个完整特化，不再引入模板参数。
struct DmMotorDefaults<DmMotorType::J4310_2EC> // 专门定义电机类型为 J4310_2EC 时的实现。此处意为以下默认值只适用于4310
{
  static DmProtocolLimits resolve_limits(float position_max_rad,
                                         float velocity_max_rad_s,
                                         float torque_max_nm)
  {
    constexpr float DEFAULT_PMAX_RAD   = 12.5f;
    constexpr float DEFAULT_VMAX_RAD_S = 30.0f;
    constexpr float DEFAULT_TMAX_NM    = 10.0f;

    return DmProtocolLimits(position_max_rad == 0.0f ? DEFAULT_PMAX_RAD : position_max_rad,
                            velocity_max_rad_s == 0.0f ? DEFAULT_VMAX_RAD_S : velocity_max_rad_s,
                            torque_max_nm == 0.0f ? DEFAULT_TMAX_NM : torque_max_nm);
  }
};

uint16_t float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
  const uint32_t span = (1UL << bits) - 1UL;
  const float scaled = (value - minimum) * static_cast<float>(span) /
                       (maximum - minimum);
  return static_cast<uint16_t>(lroundf(scaled));
}

float uint_to_float(uint16_t value, float minimum, float maximum, uint8_t bits)
{
  const uint32_t span = (1UL << bits) - 1UL;
  return static_cast<float>(value) * (maximum - minimum) /
           static_cast<float>(span) +
         minimum;
}

void write_float_le(uint8_t *destination, float value)
{
  static_assert(sizeof(float) == 4U, "DM protocol requires 32-bit float");
  memcpy(destination, &value, sizeof(value));
}

void write_u16_le(uint8_t *destination, uint16_t value)
{
  destination[0] = static_cast<uint8_t>(value);
  destination[1] = static_cast<uint8_t>(value >> 8U);
}

DmDriveState decode_drive_state(uint8_t raw_state)
{
  switch (raw_state)
  {
    case 0x0U:
      return DmDriveState::DISABLED;
    case 0x1U:
      return DmDriveState::ENABLED;
    case 0x3U:
      return DmDriveState::OUTPUT_CALIB_ERR;
    case 0x4U:
      return DmDriveState::SENSOR_ERR;
    case 0x5U:
      return DmDriveState::ENCODER_CALIB_ERR;
    case 0x8U:
      return DmDriveState::OVER_VOLTAGE;
    case 0x9U:
      return DmDriveState::UNDER_VOLTAGE;
    case 0xAU:
      return DmDriveState::OVER_CURRENT;
    case 0xBU:
      return DmDriveState::MOS_OVER_TEMP;
    case 0xCU:
      return DmDriveState::MOTOR_OVER_TEMP;
    case 0xDU:
      return DmDriveState::COMM_LOST;
    case 0xEU:
      return DmDriveState::OVERLOAD;
    default:
      return DmDriveState::UNKNOWN;
  }
}
} // namespace


template <DmMotorType TYPE>
DmMotor<TYPE> *DmMotor<TYPE>::_head = nullptr;

template <DmMotorType TYPE>
DmMotor<TYPE> *DmMotor<TYPE>::_tail = nullptr;


template <DmMotorType TYPE>
DmMotor<TYPE>::DmMotor(BspCan &can_item,
                       uint16_t esc_id,
                       uint16_t master_id,
                       DmControlMode mode,
                       float position_max_rad,
                       float velocity_max_rad_s,
                       float torque_max_nm,
                       float ratio,
                       float offset,
                       uint16_t period_ticks,
                       uint16_t phase_ticks,
                       uint16_t order)
  : _data{},
    _feedback{},
    _online(30U),
    _lvbo_data{},
    _can_item(&can_item),
    _esc_id(esc_id),
    _master_id(master_id),
    _mode(mode),
    _limits(DmMotorDefaults<TYPE>::resolve_limits(position_max_rad,
                                                  velocity_max_rad_s,
                                                  torque_max_nm)),
    _enable_state(DmEnableState::DISABLED),
    _pending_action(DmPendingAction::NONE),
    _status(Status::NOT_INIT),
    _control_msg{},
    _tx_endpoint(this,
                 &DmMotor<TYPE>::tx_callback,
                 &DmMotor<TYPE>::tx_ready_callback,
                 MotorTxManager::Schedule(period_ticks, phase_ticks, order)),
    _last_velocity(0.0f),
    _target_ready(false),
    _feedback_ready(false),
    _initialized(false),
    _next(nullptr)
{
  _data.param.offset    = offset;
  _data.param.ratio     = ratio;
  _feedback.drive_state = DmDriveState::UNKNOWN;
  _control_msg.std_id   = control_std_id();
}


template <DmMotorType TYPE>
DmMotor<TYPE>::~DmMotor()
{
  const ScopedTaskCritical lock;
  if (_initialized)
  {
    MotorTxManager::unregister_endpoint(_tx_endpoint);
    unregister_motor();
    _initialized = false;
  }
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::init(void)
{
  if (_initialized)
  {
    _status = Status::BUSY;
    return _status;
  }

  if ((_can_item == nullptr) ||
      (_esc_id > 0x7FFU) ||
      (_master_id > 0x7FFU) ||
      !mode_valid(_mode) ||
      !isfinite(_limits.position_max_rad) ||
      !isfinite(_limits.velocity_max_rad_s) ||
      !isfinite(_limits.torque_max_nm) ||
      (_limits.position_max_rad <= 0.0f) ||
      (_limits.velocity_max_rad_s <= 0.0f) ||
      (_limits.torque_max_nm <= 0.0f) ||
      !isfinite(_data.param.ratio) ||
      (_data.param.ratio <= 0.0f) ||
      !isfinite(_data.param.offset) ||
      (control_std_id() > 0x7FFU))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  if (dm_motor_feedback_in_use(*_can_item, _master_id) ||
      dji_motor_feedback_in_use(*_can_item, _master_id))
  {
    _status = Status::BUSY;
    return _status;
  }

  const ScopedTaskCritical lock;
  _status = MotorTxManager::register_endpoint(_tx_endpoint);
  if (_status != Status::OK)
  {
    return _status;
  }

  register_motor();
  _initialized = true;
  _status      = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
const MotorData &DmMotor<TYPE>::data(void) const
{
  return _data;
}


template <DmMotorType TYPE>
const DmMotorFeedback &DmMotor<TYPE>::feedback(void) const
{
  return _feedback;
}


template <DmMotorType TYPE>
const Online &DmMotor<TYPE>::online(void) const
{
  return _online;
}


template <DmMotorType TYPE>
const LuenbergerMotorData &DmMotor<TYPE>::lvbo_data(void) const
{
  return _lvbo_data;
}


template <DmMotorType TYPE>
DmDriveState DmMotor<TYPE>::drive_state(void) const
{
  return _feedback.drive_state;
}


template <DmMotorType TYPE>
DmEnableState DmMotor<TYPE>::enable_state(void) const
{
  return _enable_state;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::status(void) const
{
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::set_mit_target(float position,
                                     float velocity,
                                     float kp,
                                     float kd,
                                     float torque)
{
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_mode != DmControlMode::MIT)
  {
    _status = Status::NOT_SUPPORTED;
    return _status;
  }

  const float drive_position = position * _data.param.ratio + _data.param.offset;
  const float drive_velocity = velocity * _data.param.ratio;
  if (!isfinite(drive_position) || !isfinite(drive_velocity) ||
      !isfinite(kp) || !isfinite(kd) || !isfinite(torque) ||
      (fabsf(drive_position) > _limits.position_max_rad) ||
      (fabsf(drive_velocity) > _limits.velocity_max_rad_s) ||
      (kp < 0.0f) || (kp > 500.0f) ||
      (kd < 0.0f) || (kd > 5.0f) ||
      (fabsf(torque) > _limits.torque_max_nm))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  const uint16_t raw_position = float_to_uint(drive_position,
                                               -_limits.position_max_rad,
                                               _limits.position_max_rad,
                                               16U);
  const uint16_t raw_velocity = float_to_uint(drive_velocity,
                                               -_limits.velocity_max_rad_s,
                                               _limits.velocity_max_rad_s,
                                               12U);
  const uint16_t raw_kp = float_to_uint(kp, 0.0f, 500.0f, 12U);
  const uint16_t raw_kd = float_to_uint(kd, 0.0f, 5.0f, 12U);
  const uint16_t raw_torque = float_to_uint(torque,
                                             -_limits.torque_max_nm,
                                             _limits.torque_max_nm,
                                             12U);

  const ScopedTaskCritical lock;
  _control_msg.std_id = control_std_id();
  _control_msg.data[0] = static_cast<uint8_t>(raw_position >> 8U);
  _control_msg.data[1] = static_cast<uint8_t>(raw_position);
  _control_msg.data[2] = static_cast<uint8_t>(raw_velocity >> 4U);
  _control_msg.data[3] = static_cast<uint8_t>((raw_velocity << 4U) | (raw_kp >> 8U));
  _control_msg.data[4] = static_cast<uint8_t>(raw_kp);
  _control_msg.data[5] = static_cast<uint8_t>(raw_kd >> 4U);
  _control_msg.data[6] = static_cast<uint8_t>((raw_kd << 4U) | (raw_torque >> 8U));
  _control_msg.data[7] = static_cast<uint8_t>(raw_torque);
  _target_ready        = true;
  _status              = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::set_position_velocity_target(float position,
                                                   float velocity_limit)
{
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_mode != DmControlMode::POSITION_VELOCITY)
  {
    _status = Status::NOT_SUPPORTED;
    return _status;
  }

  const float drive_position = position * _data.param.ratio + _data.param.offset;
  const float drive_velocity = velocity_limit * _data.param.ratio;
  if (!isfinite(drive_position) || !isfinite(drive_velocity) ||
      (drive_velocity < 0.0f) ||
      (drive_velocity > _limits.velocity_max_rad_s))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  const ScopedTaskCritical lock;
  _control_msg.std_id = control_std_id();
  write_float_le(&_control_msg.data[0], drive_position);
  write_float_le(&_control_msg.data[4], drive_velocity);
  _target_ready = true;
  _status       = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::set_velocity_target(float velocity)
{
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_mode != DmControlMode::VELOCITY)
  {
    _status = Status::NOT_SUPPORTED;
    return _status;
  }

  const float drive_velocity = velocity * _data.param.ratio;
  if (!isfinite(drive_velocity) ||
      (fabsf(drive_velocity) > _limits.velocity_max_rad_s))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  const ScopedTaskCritical lock;
  _control_msg.std_id = control_std_id();
  memset(_control_msg.data, 0, sizeof(_control_msg.data));
  write_float_le(&_control_msg.data[0], drive_velocity);
  _target_ready = true;
  _status       = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::set_position_torque_target(float position,
                                                 float velocity_limit,
                                                 float current_limit_ratio)
{
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_mode != DmControlMode::POSITION_TORQUE)
  {
    _status = Status::NOT_SUPPORTED;
    return _status;
  }

  const float drive_position = position * _data.param.ratio + _data.param.offset;
  const float drive_velocity = velocity_limit * _data.param.ratio;
  if (!isfinite(drive_position) || !isfinite(drive_velocity) ||
      !isfinite(current_limit_ratio) ||
      (drive_velocity < 0.0f) || (drive_velocity > 100.0f) ||
      (current_limit_ratio < 0.0f) || (current_limit_ratio > 1.0f))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  const uint16_t raw_velocity = static_cast<uint16_t>(lroundf(drive_velocity * 100.0f));
  const uint16_t raw_current = static_cast<uint16_t>(lroundf(current_limit_ratio * 10000.0f));

  const ScopedTaskCritical lock;
  _control_msg.std_id = control_std_id();
  write_float_le(&_control_msg.data[0], drive_position);
  write_u16_le(&_control_msg.data[4], raw_velocity);
  write_u16_le(&_control_msg.data[6], raw_current);
  _target_ready = true;
  _status       = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::enable(void)
{
  const ScopedTaskCritical lock;
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_pending_action != DmPendingAction::NONE)
  {
    _status = Status::BUSY;
    return _status;
  }
  if (_enable_state == DmEnableState::ENABLED)
  {
    _status = Status::OK;
    return _status;
  }
  if ((_enable_state == DmEnableState::ENABLE_PENDING) ||
      (_enable_state == DmEnableState::WAIT_ENABLE_FEEDBACK) ||
      (_enable_state == DmEnableState::DISABLE_PENDING))
  {
    _status = Status::BUSY;
    return _status;
  }

  Status zero_target_status = Status::NOT_SUPPORTED;
  switch (_mode)
  {
    case DmControlMode::MIT:
      zero_target_status = set_mit_target(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      break;
    case DmControlMode::POSITION_VELOCITY:
      zero_target_status = set_position_velocity_target(0.0f, 0.0f);
      break;
    case DmControlMode::VELOCITY:
      zero_target_status = set_velocity_target(0.0f);
      break;
    case DmControlMode::POSITION_TORQUE:
      zero_target_status = set_position_torque_target(0.0f, 0.0f, 0.0f);
      break;
    default:
      break;
  }
  if (zero_target_status != Status::OK)
  {
    _status = zero_target_status;
    return _status;
  }

  _enable_state = DmEnableState::ENABLE_PENDING;
  _status       = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::disable(void)
{
  const ScopedTaskCritical lock;
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_enable_state == DmEnableState::DISABLE_PENDING)
  {
    _status = Status::BUSY;
    return _status;
  }

  _pending_action = DmPendingAction::NONE;
  _enable_state   = DmEnableState::DISABLE_PENDING;
  _status         = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::clear_error(void)
{
  const ScopedTaskCritical lock;
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if (_pending_action != DmPendingAction::NONE)
  {
    _status = Status::BUSY;
    return _status;
  }

  _pending_action = DmPendingAction::CLEAR_ERROR;
  _status         = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::save_current_position_as_zero(void)
{
  const ScopedTaskCritical lock;
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if ((_enable_state != DmEnableState::DISABLED) ||
      (_pending_action != DmPendingAction::NONE))
  {
    _status = Status::BUSY;
    return _status;
  }

  _pending_action = DmPendingAction::SAVE_ZERO;
  _status         = Status::OK;
  return _status;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::data_unpack(const CanRxMsg &rx)
{
  if (!_initialized)
  {
    _status = Status::NOT_INIT;
    return _status;
  }
  if ((rx.header.Identifier != _master_id) ||
      (rx.header.IdType != FDCAN_STANDARD_ID) ||
      (rx.header.RxFrameType != FDCAN_DATA_FRAME) ||
      (rx.header.DataLength != FDCAN_DLC_BYTES_8) ||
      ((rx.data[0] & 0x0FU) != (_esc_id & 0x0FU)))
  {
    _status = Status::BAD_ARG;
    return _status;
  }

  const uint16_t raw_position =
    (static_cast<uint16_t>(rx.data[1]) << 8U) | rx.data[2];
  const uint16_t raw_velocity =
    (static_cast<uint16_t>(rx.data[3]) << 4U) | (rx.data[4] >> 4U);
  const uint16_t raw_torque =
    (static_cast<uint16_t>(rx.data[4] & 0x0FU) << 8U) | rx.data[5];
  const DmDriveState drive_state = decode_drive_state(rx.data[0] >> 4U);

  const float drive_position = uint_to_float(raw_position,
                                              -_limits.position_max_rad,
                                              _limits.position_max_rad,
                                              16U);
  const float drive_velocity = uint_to_float(raw_velocity,
                                              -_limits.velocity_max_rad_s,
                                              _limits.velocity_max_rad_s,
                                              12U);
  const float output_position =
    (drive_position - _data.param.offset) / _data.param.ratio;
  const float output_velocity = drive_velocity / _data.param.ratio;
  float single_angle = fmodf(output_position, TWO_PI);
  if (single_angle < 0.0f)
  {
    single_angle += TWO_PI;
  }

  const ScopedTaskCritical lock;
  _feedback.raw_position      = raw_position;
  _feedback.raw_velocity      = raw_velocity;
  _feedback.raw_torque        = raw_torque;
  _feedback.torque            = uint_to_float(raw_torque,
                                               -_limits.torque_max_nm,
                                               _limits.torque_max_nm,
                                               12U);
  _feedback.mos_temperature   = rx.data[6];
  _feedback.rotor_temperature = rx.data[7];
  _feedback.drive_state       = drive_state;

  _data.radian_data.angle_single_round = single_angle;
  _data.radian_data.angle_multi_round  = output_position;
  _data.radian_data.velocity           = output_velocity;
  _data.radian_data.acceleration       = _feedback_ready
                                            ? (output_velocity - _last_velocity) * 1000.0f
                                            : 0.0f;
  _last_velocity = output_velocity;
  _feedback_ready = true;

  if (drive_state == DmDriveState::ENABLED)
  {
    _enable_state = DmEnableState::ENABLED;
  }
  else if (drive_state == DmDriveState::DISABLED)
  {
    _enable_state = DmEnableState::DISABLED;
  }
  else
  {
    _enable_state = DmEnableState::FAULT;
  }

  _online.refresh_task();
  return Status::OK;
}


template <DmMotorType TYPE>
uint32_t DmMotor<TYPE>::control_std_id(void) const
{
  switch (_mode)
  {
    case DmControlMode::MIT:
      return _esc_id;
    case DmControlMode::POSITION_VELOCITY:
      return 0x100U + _esc_id;
    case DmControlMode::VELOCITY:
      return 0x200U + _esc_id;
    case DmControlMode::POSITION_TORQUE:
      return 0x300U + _esc_id;
    default:
      return 0x800U;
  }
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::send_update(void)
{
  if (!_initialized || (_can_item == nullptr))
  {
    return Status::NOT_INIT;
  }

  uint8_t data[8] = {0};
  bool special_frame = false;
  uint8_t special_command = 0U;

  if (_enable_state == DmEnableState::DISABLE_PENDING)
  {
    special_frame   = true;
    special_command = 0xFDU;
  }
  else if (_enable_state == DmEnableState::ENABLE_PENDING)
  {
    special_frame   = true;
    special_command = 0xFCU;
  }
  else if (_pending_action == DmPendingAction::CLEAR_ERROR)
  {
    special_frame   = true;
    special_command = 0xFBU;
  }
  else if (_pending_action == DmPendingAction::SAVE_ZERO)
  {
    special_frame   = true;
    special_command = 0xFEU;
  }
  else if ((_enable_state == DmEnableState::ENABLED) && _target_ready)
  {
    memcpy(data, _control_msg.data, sizeof(data));
  }
  else
  {
    return Status::OK;
  }

  if (special_frame)
  {
    memset(data, 0xFF, sizeof(data));
    data[7] = special_command;
  }

  const Status send_status = _can_item->send(control_std_id(), data);
  if (send_status != Status::OK)
  {
    _status = send_status;
    return send_status;
  }

  if (_enable_state == DmEnableState::DISABLE_PENDING)
  {
    _enable_state = DmEnableState::DISABLED;
  }
  else if (_enable_state == DmEnableState::ENABLE_PENDING)
  {
    _enable_state = DmEnableState::WAIT_ENABLE_FEEDBACK;
  }

  if (_pending_action != DmPendingAction::NONE)
  {
    _pending_action = DmPendingAction::NONE;
  }

  return Status::OK;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::tx_callback(void *context)
{
  if (context == nullptr)
  {
    return Status::BAD_ARG;
  }
  return static_cast<DmMotor<TYPE> *>(context)->send_update();
}


template <DmMotorType TYPE>
bool DmMotor<TYPE>::tx_ready_callback(const void *context)
{
  if (context == nullptr)
  {
    return false;
  }

  const DmMotor<TYPE> *motor = static_cast<const DmMotor<TYPE> *>(context);
  return motor->_initialized &&
         ((motor->_enable_state == DmEnableState::ENABLE_PENDING) ||
          (motor->_enable_state == DmEnableState::DISABLE_PENDING) ||
          (motor->_enable_state == DmEnableState::ENABLED && motor->_target_ready) ||
          (motor->_pending_action != DmPendingAction::NONE));
}


template <DmMotorType TYPE>
void DmMotor<TYPE>::register_motor(void)
{
  const ScopedTaskCritical lock;
  if (_tail == nullptr)
  {
    _head = this;
    _tail = this;
  }
  else
  {
    _tail->_next = this;
    _tail        = this;
  }
}


template <DmMotorType TYPE>
void DmMotor<TYPE>::unregister_motor(void)
{
  const ScopedTaskCritical lock;
  DmMotor *previous = nullptr;
  DmMotor *current  = _head;
  while ((current != nullptr) && (current != this))
  {
    previous = current;
    current  = current->_next;
  }

  if (current == this)
  {
    if (previous == nullptr)
    {
      _head = _next;
    }
    else
    {
      previous->_next = _next;
    }
    if (_tail == this)
    {
      _tail = previous;
    }
  }
  _next = nullptr;
}


template <DmMotorType TYPE>
Status DmMotor<TYPE>::dispatch(BspCan &can_item, const CanRxMsg &rx)
{
  for (DmMotor *motor = _head; motor != nullptr; motor = motor->_next)
  {
    if ((motor->_can_item == &can_item) &&
        (rx.header.Identifier == motor->_master_id))
    {
      return motor->data_unpack(rx);
    }
  }
  return Status::BAD_ARG;
}


template <DmMotorType TYPE>
bool DmMotor<TYPE>::feedback_in_use(const BspCan &can_item, uint16_t master_id)
{
  for (DmMotor *motor = _head; motor != nullptr; motor = motor->_next)
  {
    if ((motor->_can_item == &can_item) &&
        (motor->_master_id == master_id))
    {
      return true;
    }
  }
  return false;
}


template <DmMotorType TYPE>
bool DmMotor<TYPE>::uses_can(const BspCan &can_item)
{
  for (DmMotor *motor = _head; motor != nullptr; motor = motor->_next)
  {
    if (motor->_can_item == &can_item)
    {
      return true;
    }
  }
  return false;
}


Status dm_motor_dispatch_rx(BspCan &can_item, const CanRxMsg &rx)
{
  return DmMotor<DmMotorType::J4310_2EC>::dispatch(can_item, rx);
}


bool dm_motor_feedback_in_use(const BspCan &can_item, uint16_t master_id)
{
  return DmMotor<DmMotorType::J4310_2EC>::feedback_in_use(can_item, master_id);
}


bool dm_motor_uses_can(const BspCan &can_item)
{
  return DmMotor<DmMotorType::J4310_2EC>::uses_can(can_item);
}


template class DmMotor<DmMotorType::J4310_2EC>;
