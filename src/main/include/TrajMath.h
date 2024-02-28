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

        const double v2 = m_note_max_velocity_mps * m_note_max_velocity_mps;
        const double v4 = v2 * v2;
        const double x = m_target_distance_meters;
        const double x2 = x * x;
        const double y = m_target_height_meters;
        const double g = 9.80665; // gravity

        const double gx2 = g * x2;
        const double v2y = v2 * y;
        const double gx = g * x;

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