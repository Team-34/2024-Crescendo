#pragma once

#include <frc/controller/PIDController.h>
#include "LimelightHelpers.h"
#include "SwerveDrive.h"
#include "Constants.h"
#include "TrajMath.h"

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

        LimelightUtil(TrajMath math_handler, TargetMode target_mode = TargetMode::kSpeaker);

        void Init();

        void Periodic();

        inline void SetTargetMode(TargetMode mode) { m_target_mode = mode; };

        inline double GetTargetID() { return m_target_id; }

        double m_swerve_drive_speeds[3]{};
            // 0 -> x movement
            // 1 -> y movement
            // 2 -> r movement
            // Ex: swerve_drive->Drive(m_swerve_drive_speeds[0], m_swerve_drive_speeds[1], m_swerve_drive_speeds[2]);

    };
}