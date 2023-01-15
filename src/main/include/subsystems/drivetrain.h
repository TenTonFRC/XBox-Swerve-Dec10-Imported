#pragma once

#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>
#include <units/length.h>

#include "SwerveModule.h"
#include "Constants.h"

class drivetrain : public frc2::SubsystemBase {
 public:
  drivetrain();

  void SwerveDrive(units::meters_per_second_t xSpeed,
                   units::meters_per_second_t ySpeed,
                   units::radians_per_second_t zRot,
                   bool fieldRelative);

  void UpdateOdometry();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

 private:

/*Coordinate system:

    FL (y,x)                    FR(y,-x)
                    |
                    |
                    | +y
                    |
  ------------------|---------------------
    +x              |                   -x
                    | -y
                    |
                    |
    BL(-y,x)                     BL(-y,-x)

*/

//Locations for the swerve drive modules reletive to the robot center
  frc::Translation2d m_locationFrontRight{+11.25_in, -7.25_in,};
  frc::Translation2d m_locationBackRight{-11.25_in, -7.25_in};
  frc::Translation2d m_locationFrontLeft{+11.25_in, +7.25_in};
  frc::Translation2d m_locationBackLeft{-11.25_in, +7.25_in};


  swerveModule m_FrontRight{drivetrainConstants::swerveModules::kModuleFrontRight};
  swerveModule m_BackRight{drivetrainConstants::swerveModules::kModuleBackRight};
  swerveModule m_FrontLeft{drivetrainConstants::swerveModules::kModuleFrontLeft};
  swerveModule m_BackLeft{drivetrainConstants::swerveModules::kModuleBackLeft};

  AHRS m_navX{frc::SPI::kMXP};

//Creating the kinemtics object using the module locations
  frc::SwerveDriveKinematics<4> m_kinematics{m_locationFrontRight,
                                             m_locationBackRight,
                                             m_locationFrontLeft,
                                             m_locationBackLeft};

  frc::SwerveDriveOdometry<4> m_odometry{m_kinematics, m_navX.GetRotation2d()};
};
