#pragma once

#include <frc/controller/PIDController.h>
#include "LimeLightHelpers.h"
#include "SwerveDrive.h"
#include "Constants.h"

namespace t34
{

    class LimeLightUtil
    {

    public:

        enum class TargetMode
        {
            kSpeaker,
            kAmp,
            kTrap
        };

    private:

        frc::PIDController m_ll_swerve_pid;

        std::shared_ptr<SwerveDrive> m_swerve_drive;

        std::shared_ptr<nt::NetworkTable> m_table;

        double m_tx = LimelightHelpers::getTX(LL_TABLE_NAME);
        double m_ty = LimelightHelpers::getTY(LL_TABLE_NAME);

        double m_heading_error;
        double m_kp;
        double m_min_command;

        double m_steering_adjust;
        double m_drive_x;
        double m_drive_y;

        double m_target_height_meters;
        double m_note_velocity_mps;
        double m_ll_angle_deg;
        double m_target_distance_meters;
        double m_ll_height_meters;

        TargetMode m_target_mode;

        double AdjustSteering();

    public:

        LimeLightUtil(std::shared_ptr<SwerveDrive> swerve_ptr, TargetMode target_mode = TargetMode::kSpeaker);

        void Init();

        void Periodic();

    };
}