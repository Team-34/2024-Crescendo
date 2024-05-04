#pragma once

#include "Gyro.h"
#include "subsystems/SwerveModule.h"

#include <frc2/command/SubsystemBase.h>

namespace t34 {

    class SwerveDrive : public frc2::SubsystemBase {
    public: // METHODS
        SwerveDrive();
        ~SwerveDrive() {};

        void ToggleFarisMode();
        void Drive(frc::Translation2d translation, double rotation, bool field_relative = true, bool is_open_loop = false);
        void DriveAuto(frc::ChassisSpeeds speeds);
        void Stop();


        frc::ChassisSpeeds GetRobotRelativeSpeeds();
        frc::Pose2d GetPose();
        double GetPoseX();
        double GetPoseY();
        double GetPoseRotation();

        void ResetOdometry(frc::Pose2d pose);

        std::array<frc::SwerveModuleState, 4> GetModuleStates();
        std::array<frc::SwerveModulePosition, 4> GetModulePositions();
        std::array<frc::SwerveModulePosition, 4> GetModulePositionsInverted();

        frc::Rotation2d GetYaw();

        void ResetToAbsolute();

        void Periodic() override;

        void PutTelemetry();

    private: // DATA
        bool m_field_oriented{ true };
        bool m_faris_mode{ false };
        double m_speed_scalar{ FARIS_SPEED_MODE_SCALAR };

        frc::MedianFilter<double> m_filter{ 5 };
        Gyro* m_gyro{ Gyro::Get() };

        std::array<SwerveModule, 4> m_swerve_modules = {
            SwerveModule("SM_L_FWD", ID_LEFT_FWD_DRIVE,  ID_LEFT_FWD_STEER,  ID_LEFT_FWD_CANCODER),   //,  frc::Rotation2d(units::radian_t(RotationsToRadians(LEFT_FWD_CANCODER_OFFSET)))),
            SwerveModule("SM_R_FWD", ID_RIGHT_FWD_DRIVE, ID_RIGHT_FWD_STEER, ID_RIGHT_FWD_CANCODER),  //, frc::Rotation2d(units::radian_t(RotationsToRadians(RIGHT_FWD_CANCODER_OFFSET)))),
            SwerveModule("SM_L_AFT", ID_LEFT_AFT_DRIVE,  ID_LEFT_AFT_STEER,  ID_LEFT_AFT_CANCODER),   //,  frc::Rotation2d(units::radian_t(RotationsToRadians(LEFT_AFT_CANCODER_OFFSET)))),
            SwerveModule("SM_R_AFT", ID_RIGHT_AFT_DRIVE, ID_RIGHT_AFT_STEER, ID_RIGHT_AFT_CANCODER)   //, frc::Rotation2d(units::radian_t(RotationsToRadians(RIGHT_AFT_CANCODER_OFFSET)))),
        };

        frc::SwerveDriveKinematics<4> m_swerve_drive_kinematics {
                // Translation2d objects assumes the robot is at the origin facing in the positive X direction, 
                // forward is positive X and left is positive Y.
                frc::Translation2d(units::meter_t( SWERVE_MODULE_FROM_CENTER), units::meter_t( SWERVE_MODULE_FROM_CENTER)), // Left Forward Module
                frc::Translation2d(units::meter_t( SWERVE_MODULE_FROM_CENTER), units::meter_t(-SWERVE_MODULE_FROM_CENTER)), // Right Forward Module
                frc::Translation2d(units::meter_t(-SWERVE_MODULE_FROM_CENTER), units::meter_t( SWERVE_MODULE_FROM_CENTER)), // Left Aft Module
                frc::Translation2d(units::meter_t(-SWERVE_MODULE_FROM_CENTER), units::meter_t(-SWERVE_MODULE_FROM_CENTER))  // Right Aft Module                                                                  
            };

        frc::SwerveDriveOdometry<4> m_swerve_odometry { m_swerve_drive_kinematics, frc::Rotation2d(units::degree_t(m_gyro->GetAngle())), 
                {
                    frc::SwerveModulePosition{ m_swerve_modules[0].GetPosition().distance, m_swerve_modules[0].GetCanCoder() },
                    frc::SwerveModulePosition{ m_swerve_modules[1].GetPosition().distance, m_swerve_modules[1].GetCanCoder() },
                    frc::SwerveModulePosition{ m_swerve_modules[2].GetPosition().distance, m_swerve_modules[2].GetCanCoder() },
                    frc::SwerveModulePosition{ m_swerve_modules[3].GetPosition().distance, m_swerve_modules[3].GetCanCoder() },
                }
            };

    };

}
