#pragma once

#include <math.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "LimelightHelpers.h"

namespace t34
{
    class TrajMath
    {
        double m_note_max_velocity_mps = 11.884;
        double m_motor_output = 1;
        double m_target_distance_meters;
        double m_target_height_meters;
        double m_apriltag_height_meters;
        double m_limelight_height_meters;
        double m_shooter_angle_degrees;
        double m_limelight_angle_degrees;

        double m_target_tx;
        double m_target_ty;

        const double g = 9.80665; // gravity


    public:

        TrajMath
        (
            double note_max_velocity_mps,
            double target_height_meters,
            double apriltag_height_meters,
            double limelight_height_meters,
            double shooter_angle,
            double limelight_angle
        );

        void Periodic();

        void PutTelemetry();

        double GetFiringAngleDeg() const;
        inline units::degree_t GetFiringAngle() const { return units::degree_t(GetFiringAngleDeg()); }

        bool IsInRange() const;

        double GetDistanceFromTarget() const;

        inline void InputMotorOutputPercent(const double percent) { m_motor_output = percent; }

        inline void SetTargetHeightMeters(const double meters) { m_target_height_meters = meters; }

        inline void SetAprilTagHeightMeters(const double meters) { m_apriltag_height_meters = meters; }


    };
}