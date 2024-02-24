#include "subsystems/LimeLightUtil.h"

double t34::LimeLightUtil::AdjustSteering()
{
    if (fabs(m_heading_error) > 1.0) {

        if (m_heading_error < 0.0) {

            m_steering_adjust = m_kp*m_heading_error + m_min_command;

        }
        else 
        {

            m_steering_adjust = m_kp*m_heading_error - m_min_command;
      
    }
  }
}

t34::LimeLightUtil::LimeLightUtil(std::shared_ptr<SwerveDrive> swerve_ptr, TargetMode target_mode)
:   m_ll_swerve_pid(0.1, 0.0, 0.1),
    m_swerve_drive(swerve_ptr),
    m_table(LimelightHelpers::getLimelightNTTable(LL_TABLE_NAME)),
    m_tx(LimelightHelpers::getTX(LL_TABLE_NAME)),
    m_ty(LimelightHelpers::getTY(LL_TABLE_NAME)),
    m_heading_error(-m_tx),
    m_kp(-0.1),
    m_min_command(0.5),
    m_steering_adjust(0.0),
    m_drive_x(0.0),
    m_drive_y(0.0),
    m_target_height_meters(0.0),
    m_note_velocity_mps(0.0),
    m_ll_angle_deg(90.0),
    m_target_distance_meters(0.0),
    m_ll_height_meters(0.0),
    m_target_mode(target_mode) {}
