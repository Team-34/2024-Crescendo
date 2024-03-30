#pragma once

#include <frc/controller/PIDController.h>

#include <frc/DriverStation.h>
#include "LimelightHelpers.h"
#include "SwerveDrive.h"
#include "Constants.h"
#include "TrajMath.h"
#include "commands/ControllerDriveCommand.h"

namespace t34
{

    class LimelightUtil
    {

    public:

        enum class TargetMode
        {
            kNone = -1,

            kSpeaker,
            kAmp,
            kTrap
        };

    private:

        struct SwerveSpeeds
        {
            double x, y, r;
        };

        frc::PIDController m_limelight_swerve_pid;

        std::shared_ptr<nt::NetworkTable> m_table;

        double m_tx = LimelightHelpers::getTX(LIMELIGHT_TABLE_NAME);
        double m_ty = LimelightHelpers::getTY(LIMELIGHT_TABLE_NAME);

        double m_heading_error;
        double m_kp;
        double m_min_command;

        double m_steering_adjust;
        double m_drive_x;
        double m_drive_y;

        double m_current_id;
        double m_target_id;

        TargetMode m_target_mode;

        void AdjustSteering();

    public:

        TrajMath m_math_handler;
        SwerveSpeeds m_swerve_drive_speeds;

        LimelightUtil(TrajMath math_handler, TargetMode target_mode = TargetMode::kSpeaker);

        void Init();

        void Periodic();

        //inline void SetTargetMode(TargetMode mode) { m_target_mode = mode; }

        inline void TargetSpeaker() { m_target_mode = TargetMode::kSpeaker; }
        inline void TargetAmp() { m_target_mode = TargetMode::kAmp; }
        inline void TargetTrap() { m_target_mode = TargetMode::kTrap; }

        inline double GetTargetID() const { return m_target_id; }

        inline bool IsTargetAcquired() const { return m_target_id != -1.0; }

    };
}