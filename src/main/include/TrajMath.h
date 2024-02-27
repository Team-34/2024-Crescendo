#pragma once

#include <math.h>

#include "Constants.h"

namespace t34
{
    class TrajMath
    {
        double m_note_max_velocity_mps;
        double m_target_distance_meters;
        double m_target_height_meters;
        double m_limelight_height_meters;
        double m_shooter_angle;
        double m_limelight_angle;

        double m_target_tx;
        double m_target_ty;

    public:
    
        double GetFiringAngle();

        bool IsInRange();

        double GetDistanceFromTarget();


    };
}