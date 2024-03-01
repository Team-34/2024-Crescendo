#pragma once

#include <memory>
#include <chrono>

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <units/length.h>
#include <units/math.h>
#include <frc/MathUtil.h>
#include "commands/ControllerDriveCommand.h"
#include <frc/filter/SlewRateLimiter.h>

#include "subsystems/SwerveDrive.h"
#include "T34Controller.hpp"

namespace t34 {

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

        frc::SlewRateLimiter<units::scalar> m_x_speed_limiter {3 / 1_s};
        frc::SlewRateLimiter<units::scalar> m_y_speed_limiter {3 / 1_s};
        frc::SlewRateLimiter<units::scalar> m_rotation_limiter{3 / 1_s};


        std::chrono::time_point<std::chrono::steady_clock> m_last_zero;
    };

}

