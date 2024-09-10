// the motors we use inherit from the Motor struct.
// therefore, we can expect that all of the motors can
// perform the functions "get_angle", "get_angular_velocity",
// "set_angle", "get_angular_velocity".
// rotation units for angle is rotations
// rotation units for angular velocity is rotations/sec.
// todo: talon srx.

#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <frc/geometry/Rotation2d.h>
#include <rev/CANSparkMax.h>
#include <stdexcept>
#include <stdio.h>

namespace hardware {

class Motor {
public:
  virtual void set_angle(units::angle::turn_t target) {};
  virtual void set_angular_velocity(units::angular_velocity::turns_per_second_t target) {};
  virtual units::angle::turn_t get_angle() {};
  virtual units::angular_velocity::turns_per_second_t get_angular_velocity() {};

  template <typename T> T *get_motor_config() { return nullptr; };
};

// talon fx motor controller
class TalonFX : Motor {
public:
  TalonFX(int idx)
      : m_fx{idx, std::string("rio")}, m_config{}, m_voltage_control{0_tr},
        v_voltage_control{0_tps} {
    m_config.Slot1.kP = 0.11;
    m_config.Slot1.kI = 0.5;
    m_config.Slot1.kD = 0.0001;
    m_config.Slot1.kV = 0.12;
    m_config.CurrentLimits.SupplyCurrentLimit = 35;
    m_config.CurrentLimits.SupplyCurrentLimitEnable = true;
    apply_config();
    m_fx.SetPosition(0_tr);
    m_voltage_control.Slot = 0;
    m_voltage_control.EnableFOC = false;
    v_voltage_control.Slot = 1;
    v_voltage_control.EnableFOC = false;
  }
  TalonFX(int idx, ctre::phoenix6::configs::TalonFXConfiguration config)
      : m_fx{idx, std::string("rio")}, m_voltage_control{0_tr},
        v_voltage_control{0_tps} {
    m_config = config;
    apply_config();
    m_fx.SetPosition(0_tr);
    m_voltage_control.Slot = 0;
    m_voltage_control.EnableFOC = false;
    v_voltage_control.Slot = 1;
    v_voltage_control.EnableFOC = false;
  }

  ctre::phoenix6::configs::TalonFXConfiguration *get_motor_config() {
    return &m_config;
  }

  // apply talonfx config to the motor.
  void apply_config() {
    ctre::phoenix::StatusCode status =
        ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    for (int i = 0; i < 5; i++) { // try to apply config 5 times
      status = m_fx.GetConfigurator().Apply(m_config);
      if (status.IsOK())
        break;
    }
    if (!status.IsOK())
      throw std::invalid_argument("failed to apply config to motor.");
  };

  void set_angle(units::angle::turn_t target) override;
  void set_angular_velocity(
      units::angular_velocity::turns_per_second_t target) override;
  units::angle::turn_t get_angle() override;
  units::angular_velocity::turns_per_second_t get_angular_velocity() override;

private:
  ctre::phoenix6::hardware::TalonFX m_fx;
  ctre::phoenix6::configs::TalonFXConfiguration m_config;
  ctre::phoenix6::controls::MotionMagicVoltage m_voltage_control;
  ctre::phoenix6::controls::VelocityVoltage v_voltage_control;
};

// // spark max motor controller
class SparkMax : Motor {
public:
  SparkMax(int idx)
      : m_spark_max{idx, rev::CANSparkMax::MotorType::kBrushless} {
    m_spark_max.GetPIDController().SetP(0.7);
  }
  SparkMax(int idx, rev::CANSparkMax::MotorType motor_type)
      : m_spark_max{idx, motor_type} {
    m_spark_max.GetPIDController().SetP(0.7);
  }
  rev::CANSparkMax *get_motor_config() { return &m_spark_max; };

  void set_angle(units::angle::turn_t target) override;
  void set_angular_velocity(
      units::angular_velocity::turns_per_second_t target) override;
  units::angle::turn_t get_angle() override;
  units::angular_velocity::turns_per_second_t get_angular_velocity() override;

private:
  rev::CANSparkMax m_spark_max;
};

class SimMotor : Motor {
public:
  void set_angle(units::angle::turn_t target_angle) { m_angle = target_angle; }
  void set_angular_velocity(units::angular_velocity::turns_per_second_t target_velocity) { m_velocity = target_velocity; }
  units::angle::turn_t get_angle() { return m_angle; }
  units::angular_velocity::turns_per_second_t get_angular_velocity() { return m_velocity; }

private:
  units::angle::turn_t m_angle;
  units::angular_velocity::turns_per_second_t m_velocity;
};

} // namespace hardware
