#pragma once

#include <vector>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

#include "subsystems/swerve/module.h"

namespace swerve {

class BaseSwerveIO {
public:
  virtual void set_speeds(frc::ChassisSpeeds speeds) {};
  virtual void set_pose(frc::Pose2d pose) {};
  virtual void periodic() {};
  virtual void set_angle(frc::Rotation2d angle) {};
  virtual frc::ChassisSpeeds get_speeds() {};
  virtual frc::Pose2d get_pose() {};
  virtual std::vector<frc::SwerveModuleState> get_states() {};
};

class RealSwerve : BaseSwerveIO {

};

class SimSwerve : BaseSwerveIO {

};

}