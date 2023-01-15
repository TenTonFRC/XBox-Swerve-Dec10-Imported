#include "subsystems/drivetrain.h"

#include <frc/smartdashboard/SmartDashboard.h>

drivetrain::drivetrain() {
    m_navX.ZeroYaw();
}

void drivetrain::SwerveDrive(units::meters_per_second_t xSpeed,
                             units::meters_per_second_t ySpeed,
                             units::radians_per_second_t zRot,
                             bool fieldRelative) {
    auto moduleStates = m_kinematics.ToSwerveModuleStates(
        fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                            xSpeed, ySpeed, zRot, m_navX.GetRotation2d())
                      : frc::ChassisSpeeds{xSpeed, ySpeed, zRot});
    m_kinematics.DesaturateWheelSpeeds(&moduleStates, drivetrainConstants::calculations::kModuleMaxSpeed);

    frc::SmartDashboard::PutNumber("xSpeed", xSpeed.value());
    frc::SmartDashboard::PutNumber("ySpeed", ySpeed.value());
    frc::SmartDashboard::PutNumber("zRotation", zRot.value());
    
    auto [FrontRight, BackRight, FrontLeft, BackLeft] = moduleStates;

    m_FrontRight.SetDesiredState(FrontRight);
    m_BackRight.SetDesiredState(BackRight);
    m_FrontLeft.SetDesiredState(FrontLeft);
    m_BackLeft.SetDesiredState(BackLeft);
}

void drivetrain::UpdateOdometry() {
    m_odometry.Update(m_navX.GetRotation2d(), m_FrontRight.GetState(),
                      m_BackRight.GetState(), m_FrontLeft.GetState(),
                      m_BackLeft.GetState());
}

void drivetrain::Periodic() {
    UpdateOdometry();

    // Test posting angle to Dashboard.
    frc::SmartDashboard::PutNumber("Front Right Angle", m_FrontRight.DashboardInfo(swerveModule::DataType::kCurrentAngle));
    frc::SmartDashboard::PutNumber("Back Right Angle", m_BackRight.DashboardInfo(swerveModule::DataType::kCurrentAngle));
    frc::SmartDashboard::PutNumber("Front Left Angle", m_FrontLeft.DashboardInfo(swerveModule::DataType::kCurrentAngle));
    frc::SmartDashboard::PutNumber("Back Left Angle", m_BackLeft.DashboardInfo(swerveModule::DataType::kCurrentAngle));

    frc::SmartDashboard::PutNumber("Front Right TARGET", m_FrontRight.DashboardInfo(swerveModule::DataType::kTargetAngle));
    frc::SmartDashboard::PutNumber("Back Right TARGET", m_BackRight.DashboardInfo(swerveModule::DataType::kTargetAngle));
    frc::SmartDashboard::PutNumber("Front Left TARGET", m_FrontLeft.DashboardInfo(swerveModule::DataType::kTargetAngle));
    frc::SmartDashboard::PutNumber("Back Left TARGET", m_BackLeft.DashboardInfo(swerveModule::DataType::kTargetAngle));
}

void drivetrain::SimulationPeriodic() {}
