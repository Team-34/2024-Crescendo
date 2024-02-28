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

        const double g = 9.80665; // gravity


    public:

        TrajMath
        (
            double note_max_velocity_mps,
            double target_height_meters,
            double limelight_height_meters,
            double shooter_angle,
            double limelight_angle
        );

        double GetFiringAngleDeg();

        bool IsInRange();

        double GetDistanceFromTarget();


    };
}