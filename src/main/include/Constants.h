#pragma once

#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <cmath>
#include <wpi/numbers>


/**
 * This header contains hold robot-wide numerical or boolean constants ONLY.
 * 
 * Place constants into subsystem/command -specific NAMESPACES within this
 * header, which can then be included (where they are needed).
 */

namespace controllerConstants {
    //USB port addresses on drivestation PC.
    constexpr int kControllerMainID = 0;
    constexpr int kControllerAuxID = 1;
}

namespace drivetrainConstants {
    //CAN IDs
    constexpr int kMotorDriveFrontRightID = 1;
    constexpr int kMotorDriveBackRightID = 4;
    constexpr int kMotorDriveFrontLeftID = 3;
    constexpr int kMotorDriveBackLeftID = 2;

    constexpr int kMotorTurnFrontRightID = 5;
    constexpr int kMotorTurnBackRightID = 8;
    constexpr int kMotorTurnFrontLeftID = 7;
    constexpr int kMotorTurnBackLeftID = 6;

    constexpr int kEncoderTurnFrontRightID = 9;
    constexpr int kEncoderTurnBackRightID = 12;
    constexpr int kEncoderTurnFrontLeftID = 11;
    constexpr int kEncoderTurnBackLeftID = 10;

    constexpr double kFrontRight{0};
    constexpr double kBackRight{0};
    constexpr double kFrontLeft{0};
    constexpr double kBackLeft{0};


    namespace swerveModules {
        constexpr double kModuleFrontRight[4]{kMotorDriveFrontRightID,
                                                   kMotorTurnFrontRightID,
                                                   kEncoderTurnFrontRightID,
                                                   kFrontRight};
        constexpr double kModuleBackRight[4]{kMotorDriveBackRightID,
                                                  kMotorTurnBackRightID,
                                                  kEncoderTurnBackRightID,
                                                  kBackRight};
        constexpr double kModuleFrontLeft[4]{kMotorDriveFrontLeftID,
                                                  kMotorTurnFrontLeftID,
                                                  kEncoderTurnFrontLeftID,
                                                  kFrontLeft};
        constexpr double kModuleBackLeft[4]{kMotorDriveBackLeftID,
                                                 kMotorTurnBackLeftID,
                                                 kEncoderTurnBackLeftID,
                                                 kBackLeft};
    }

    namespace calculations {
        constexpr auto kFinalDriveRatio{6.75 * 360_deg};
        constexpr units::length::inch_t kWheelCircumference = {2 * M_PI * 3.8_in / 2};

        constexpr auto kModuleMaxSpeed{16.3_fps};
        constexpr auto kChassisMaxSpeed{16.3_fps};

        constexpr auto kModuleMaxAngularVelocity{M_PI * 1_rad_per_s};  // radians per second
        constexpr auto kModuleMaxAngularAcceleration{M_PI * 2_rad_per_s / 1_s};  // radians per second^2

        constexpr double kMotorMaxOutput = 0.5;
        constexpr double kMotorDeadband = 0.1;
    }
}
