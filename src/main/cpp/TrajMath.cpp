#include "TrajMath.h"

t34::TrajMath::TrajMath
(
    double note_max_velocity_mps,
    double target_height_meters,
    double apriltag_height_meters,
    double limelight_height_meters,
    double shooter_angle,
    double limelight_angle
):
    m_note_max_velocity_mps(note_max_velocity_mps),
    m_target_height_meters(target_height_meters),
    m_apriltag_height_meters(apriltag_height_meters),
    m_limelight_height_meters(limelight_height_meters),
    m_shooter_angle_degrees(shooter_angle),
    m_limelight_angle_degrees(limelight_angle),
    m_target_tx(0.0),
    m_target_ty(0.0),
    m_previous_firing_angle(90.0)
{}     

void t34::TrajMath::Periodic()
{
    m_target_ty = LimelightHelpers::getTY(LIMELIGHT_TABLE_NAME);
    m_target_tx = LimelightHelpers::getTX(LIMELIGHT_TABLE_NAME);
    m_limelight_angle_degrees = m_target_ty * LIMELIGHT_DEGREE_SCALAR;
}

void t34::TrajMath::PutTelemetry()
{
    frc::SmartDashboard::PutNumber("Limelight degree: ", m_limelight_angle_degrees);
    frc::SmartDashboard::PutNumber("Target height: ", (m_target_height_meters - m_limelight_height_meters) );
    frc::SmartDashboard::PutNumber("Arm Firing Angle (degrees): ", GetArmFiringAngleDeg());
    frc::SmartDashboard::PutNumber("TrajMath tx: ", m_target_tx);
    frc::SmartDashboard::PutNumber("TrajMath ty: ", m_target_ty);
}

double t34::TrajMath::GetArmFiringAngleDeg()
{
    //                   ____________________
    //          ⎛  v² - √ v⁴ - g(gx² + 2v²y)   ⎞
    // θ = atan ⎜ ———————————————————————————— ⎟
    //          ⎝               gx             ⎠
    //
    // Source:
    //   Solving Ballistic Trajectories <https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/>
    //   See section “Firing Angle to Hit Stationary Target.”

    //const auto v = m_note_max_velocity_mps * 0.8;
    //const auto v2 = v * v;
    //const auto v4 = v2 * v2;
    const auto x = GetDistanceFromTarget();
    const auto x2 = x * x;
    //const auto y  = m_target_height_meters;
//
    //const auto gx  = g * x;
    //const auto gx2 = g * x2;
    //const auto v2y = v2 * y;
//
    //const auto numerator = v2 - sqrt(v4 - (g * (gx2 + (2*v2y))));
    //const auto denominator = gx;
    //const auto θ = atan(numerator / denominator);

    //frc::SmartDashboard::PutNumber("v", v);
    //frc::SmartDashboard::PutNumber("v2", v2);
    //frc::SmartDashboard::PutNumber("v4", v4);
    //frc::SmartDashboard::PutNumber("x", x);
    //frc::SmartDashboard::PutNumber("x2", x2);
    //frc::SmartDashboard::PutNumber("y", y);
    //frc::SmartDashboard::PutNumber("gx", gx);
    //frc::SmartDashboard::PutNumber("gx2", gx2);
    //frc::SmartDashboard::PutNumber("v2y", v2y);
    //frc::SmartDashboard::PutNumber("Numerator", numerator);
    //frc::SmartDashboard::PutNumber("Denominator", denominator);

    double θ = (-1.9171 * x2) + (17.186 * x) + (3.5412);

    if (std::isnan(θ))
    {
        return m_previous_firing_angle;
    }

    m_previous_firing_angle = θ;

    return θ;

    //m_previous_firing_angle = std::clamp((SHOOTER_OFFSET_ANGLE_DEG - RAD_TO_DEG(θ)), 0.0, 90.0);

    //return std::clamp((SHOOTER_OFFSET_ANGLE_DEG - RAD_TO_DEG(θ)), 0.0, 90.0);
}

bool t34::TrajMath::IsInRange() const
{
    const double θ = DEG_TO_RAD(m_shooter_angle_degrees);
    const double v2 = m_note_max_velocity_mps * m_note_max_velocity_mps;
    const double x = m_target_distance_meters;
    const double cosθ = cos(θ);
    const double cos2θ = cosθ * cosθ;

    const double rate_of_rise = -(((2 * g) / (2 * v2 * cos2θ)) * x) + tan(θ);

    return rate_of_rise > 0; 
}

double t34::TrajMath::GetDistanceFromTarget() const
{
    return (
        //(m_target_height_meters - m_limelight_height_meters) / tan( DEG_TO_RAD(m_limelight_angle_degrees) )
        log2( (2.5 / LimelightHelpers::getTA())) - 0.3
    );
}