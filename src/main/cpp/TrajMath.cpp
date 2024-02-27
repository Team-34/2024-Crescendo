#include "TrajMath.h"

double t34::TrajMath::GetFiringAngle()
{
    return 
    (
        _180_DIV_PI *
        atan(
            (pow(m_note_max_velocity_mps, 2) - 
            sqrt(
                pow(m_note_max_velocity_mps, 4) - 9.807 * 
                (
                    (9.807 * pow(m_target_distance_meters, 2)) + (2 * pow(m_note_max_velocity_mps, 2) * m_target_height_meters)
                )
            )) / (9.807 * m_target_distance_meters)
        )
    );
}

bool t34::TrajMath::IsInRange()
{
    double rate = (
        tan(m_shooter_angle) - 
        (9.807 * m_target_distance_meters) / (pow(m_shooter_angle, 2) * pow(m_note_max_velocity_mps, 2))
    );

    if (rate > 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double t34::TrajMath::GetDistanceFromTarget()
{
    return ( 
        (m_target_height_meters - m_limelight_height_meters) / tan(
            RAD_TO_DEG( (m_limelight_angle + m_target_ty) ) 
        )
    );
}