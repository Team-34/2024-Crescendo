#pragma once

#include <memory>
#include <chrono>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/SwerveDrive.h"
#include "T34Controller.hpp"

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

    private:
        std::shared_ptr<SwerveDrive> m_swerve_drive;
        std::shared_ptr<T34XboxController> m_controller;
        double m_driving_speed;

        frc::SlewRateLimiter<units::scalar> m_x_limiter;
        frc::SlewRateLimiter<units::scalar> m_y_limiter;
        frc::SlewRateLimiter<units::scalar> m_r_limiter;

        std::chrono::time_point<std::chrono::steady_clock> m_last_zero;
    };

}

