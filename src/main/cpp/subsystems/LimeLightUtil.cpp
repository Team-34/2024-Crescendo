#include "subsystems/LimelightUtil.h"

t34::LimelightUtil::LimelightUtil(TrajMath math_handler, TargetMode target_mode)
:   m_limelight_swerve_pid(0.1, 0.0, 0.1),
    m_table(LimelightHelpers::getLimelightNTTable(LIMELIGHT_TABLE_NAME)),
    m_tx(LimelightHelpers::getTX(LIMELIGHT_TABLE_NAME)),
    m_ty(LimelightHelpers::getTY(LIMELIGHT_TABLE_NAME)),
    m_heading_error(-m_tx),
    m_kp(-0.1),
    m_min_command(0.5),
    m_steering_adjust(0.0),
    m_drive_x(0.0),
    m_drive_y(0.0),
    m_swerve_drive_speeds{0.0, 0.0, 0.0},
    m_math_handler(math_handler),
    m_current_id(-1.0),
    m_target_id(-1.0),
    m_target_mode(target_mode) 
    
{}

void t34::LimelightUtil::AdjustSteering()
{
    if (fabs(m_heading_error) > 1.0) {

        m_steering_adjust = m_heading_error < 0.0 ?
            m_kp * m_heading_error + m_min_command :
            m_kp * m_heading_error - m_min_command;
    }
}

void t34::LimelightUtil::Periodic()
{
    m_table = LimelightHelpers::getLimelightNTTable(LIMELIGHT_TABLE_NAME);
    m_tx = LimelightHelpers::getTX(LIMELIGHT_TABLE_NAME);
    m_ty = LimelightHelpers::getTY(LIMELIGHT_TABLE_NAME);
    m_current_id = LimelightHelpers::getFiducialID(LIMELIGHT_TABLE_NAME);
    m_math_handler.Periodic();

    switch (m_target_mode)
    {
        case (TargetMode::kAmp):
            m_math_handler.SetTargetHeightMeters(0.864);
            m_math_handler.SetApriltagHeightMeters(1.45);
            
            m_target_id = (m_current_id == 5 || m_current_id == 6) ? m_current_id : -1.0;

            break;
        
        case (TargetMode::kSpeaker):
            m_math_handler.SetTargetHeightMeters(1.9815);
            m_math_handler.SetApriltagHeightMeters(1.435);

            m_target_id = ( m_current_id == 1 ||
                            m_current_id == 2 ||
                            m_current_id == 7 ||
                            m_current_id == 8) ? m_current_id : -1.0;

            break;
    }

    m_heading_error = std::clamp(-m_tx, -1.0, 1.0);
    m_drive_x = std::clamp((fabs(m_tx) / 25.0), -1.0, 1.0);
    m_drive_y = m_math_handler.IsInRange() ? 0.0 : 0.5;

    m_swerve_drive_speeds[0] = m_drive_x;
    m_swerve_drive_speeds[1] = m_drive_y;
    m_swerve_drive_speeds[2] = m_steering_adjust;

    

}