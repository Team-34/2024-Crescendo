#pragma once

#include <memory>
#include <chrono>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/SwerveDrive.h"
#include "T34Controller.hpp"
#include <frc/filter/SlewRateLimiter.h>

namespace t34 {

    double ScaleToRange(double x, double in_min, double in_max, double out_min, double out_max);

    class ControllerDriveCommand
        : public frc2::CommandHelper<frc2::Command, ControllerDriveCommand> {

    public:
        ControllerDriveCommand(std::shared_ptr<SwerveDrive> drive, std::shared_ptr<T34XboxController> controller);

        void Initialize();
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;
        void JoystickSlewRateLimiter();

    private:
        std::shared_ptr<SwerveDrive> m_swerve_drive;
        std::shared_ptr<T34XboxController> m_controller;
        double m_driving_speed;

        std::chrono::time_point<std::chrono::steady_clock> m_last_zero;

        frc::SlewRateLimiter<units::scalar> m_x_speed_limiter{0.2 / 1_s};
        frc::SlewRateLimiter<units::scalar> m_y_speed_limiter{0.2 / 1_s};
        frc::SlewRateLimiter<units::scalar> m_rot_speed_limiter{0.2 / 1_s};
    };

}

