#pragma once

#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include "hardware/motor.h"

namespace swerve {

struct SwerveModuleConfig {
  frc::Translation2d translation;
  units::length::inch_t wheel_diameter;
  double drive_ratio;
  double angle_ratio;
};

class SwerveModule {
  SwerveModule(
		hardware::Motor drive_motor,
		hardware::Motor angle_motor,
    SwerveModuleConfig config
	) : m_drive_motor{drive_motor}, m_angle_motor{angle_motor}, m_config{config} {}
  void drive(frc::SwerveModuleState inital_target_state);
  frc::SwerveModuleState get_state();
  /// @returns the rotational velocity (angle motor) of the module
  units::angular_velocity::turns_per_second_t get_angular_velocity();
  frc::SwerveModulePosition get_position();
	frc::Translation2d get_translation_from_center();
  void set_module_angle(units::angle::radian_t target_angle);
  void set_module_velocity(units::velocity::meters_per_second_t target_velocity);

private:
  hardware::Motor m_drive_motor;
  hardware::Motor m_angle_motor;
  SwerveModuleConfig m_config;
};

}
