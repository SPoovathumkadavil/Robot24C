
#include "hardware/motor.h"

void hardware::TalonFX::set_angle(units::angle::turn_t target) {
  m_fx.SetControl(m_voltage_control.WithPosition(target));
}
void hardware::TalonFX::set_angular_velocity(
    units::angular_velocity::turns_per_second_t target) {
  m_fx.SetControl(v_voltage_control.WithVelocity(target));
}
units::angle::turn_t hardware::TalonFX::get_angle() {
  return m_fx.GetPosition().GetValue();
}
units::angular_velocity::turns_per_second_t
hardware::TalonFX::get_angular_velocity() {
  return m_fx.GetVelocity().GetValue();
}

void hardware::SparkMax::set_angle(units::angle::turn_t target) {
  m_spark_max.GetPIDController().SetReference(
      target.value(), rev::CANSparkBase::ControlType::kPosition);
}
void hardware::SparkMax::set_angular_velocity(
    units::angular_velocity::turns_per_second_t target) {
  m_spark_max.GetPIDController().SetReference(
      (target * 60).value(), rev::CANSparkBase::ControlType::kVelocity);
}
units::angle::turn_t hardware::SparkMax::get_angle() {
  return units::angle::turn_t{m_spark_max.GetEncoder().GetPosition()};
}
units::angular_velocity::turns_per_second_t
hardware::SparkMax::get_angular_velocity() {
  return units::angular_velocity::turns_per_second_t{
      m_spark_max.GetEncoder().GetVelocity() / 60};
}
