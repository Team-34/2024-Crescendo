#pragma once

#include "subsystems/SwerveConstants.h"

using namespace ctre::phoenix6;
using namespace ctre::phoenix6::hardware;

namespace t34 {

    // getRotations
    inline double RadiansToRotations(double radians) { return radians / PI2; }
    inline double DegreesToRotations(double degrees) { return RadiansToRotations(DEG_TO_RAD(degrees)); }
    
    // fromRotations
    inline double RotationsToRadians(double rotations) { return rotations * PI2; }

    class SwerveModule {
    public:
        SwerveModule(std::string name, const int drive_id, const int steer_id, const int cancoder_id);

        inline void SetDriveBrake(bool on = true) { m_drive->SetNeutralMode(on ? signals::NeutralModeValue::Brake : signals::NeutralModeValue::Coast); }

        void Stop();
        void SetDesiredState(frc::SwerveModuleState desired_state, bool is_open_loop);
        void SetSpeed(frc::SwerveModuleState desired_state, bool is_open_loop);
        void SetAngle(frc::SwerveModuleState desired_state);
        void SetAngle(frc::Rotation2d angle);
        frc::Rotation2d GetAngle();
        frc::Rotation2d GetAngleForOdometry();
        frc::Rotation2d GetCanCoder();
        void ResetToAbsolute();
        frc::SwerveModuleState GetState();
        frc::SwerveModulePosition GetPosition();

        void PutTelemetry();

    private: // DATA
        std::string m_module_name;
        std::shared_ptr<ctre::phoenix6::hardware::TalonFX> m_drive;
        std::shared_ptr<ctre::phoenix6::hardware::TalonFX> m_steer;
        std::shared_ptr<ctre::phoenix6::hardware::core::CoreCANcoder> m_cancoder;

        frc::Rotation2d m_last_angle;

        controls::VelocityVoltage m_drive_velocity_voltage{ units::angular_velocity::turns_per_second_t(0) };
        controls::PositionVoltage m_steer_position_voltage{ units::angle::turn_t(0) };
    };
}

