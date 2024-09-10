
#include "subsystems/swerve/module.h"

void swerve::SwerveModule::drive(frc::SwerveModuleState inital_target_state) {
  frc::SwerveModuleState target_state =
      frc::SwerveModuleState::Optimize(inital_target_state, get_state().angle);
  set_module_velocity(target_state.speed *
                      abs((target_state.angle - get_state().angle).Cos()));
  set_module_angle(target_state.angle.Radians());
}

frc::SwerveModuleState swerve::SwerveModule::get_state() {
  return frc::SwerveModuleState{
      units::meters_per_second_t{(m_drive_motor.get_angular_velocity() *
                                 m_config.drive_ratio * M_PI *
                                 m_config.wheel_diameter).value()},
      frc::Rotation2d{m_angle_motor.get_angle() * m_config.angle_ratio}};
}

units::angular_velocity::turns_per_second_t
swerve::SwerveModule::get_angular_velocity() {
  return m_angle_motor.get_angular_velocity() * m_config.angle_ratio;
}

frc::SwerveModulePosition swerve::SwerveModule::get_position() {
  return frc::SwerveModulePosition{
      units::meter_t{(m_drive_motor.get_angle() * m_config.drive_ratio *
                     m_config.wheel_diameter * M_PI).value()},
      get_state().angle};
}

frc::Translation2d swerve::SwerveModule::get_translation_from_center() {
  return m_config.translation;
}

void swerve::SwerveModule::set_module_angle(
    units::angle::radian_t target_angle) {
  m_angle_motor.set_angle(
      units::angle::turn_t{target_angle / m_config.angle_ratio});
}

void swerve::SwerveModule::set_module_velocity(
    units::velocity::meters_per_second_t target_velocity) {
  m_drive_motor.set_angular_velocity(
      units::angular_velocity::turns_per_second_t{
          (target_velocity * 2 /
           (m_config.drive_ratio * m_config.wheel_diameter))
              .value()});
}
