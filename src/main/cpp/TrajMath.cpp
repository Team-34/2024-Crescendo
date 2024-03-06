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
    m_target_ty(0.0)
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
    frc::SmartDashboard::PutNumber("TrajMath tx: ", m_target_tx);
    frc::SmartDashboard::PutNumber("TrajMath ty: ", m_target_ty);
}

double t34::TrajMath::GetFiringAngleDeg()
{
    //                   ____________________
    //          ⎛  v² - √ v⁴ - g(gx² + 2v²y)   ⎞
    // θ = atan ⎜ ———————————————————————————— ⎟
    //          ⎝               gx             ⎠
    //
    // Source:
    //   Solving Ballistic Trajectories <https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/>
    //   See section “Firing Angle to Hit Stationary Target.”

    const auto v = m_note_max_velocity_mps * m_motor_output;
    const auto v2 = v * v;
    const auto v4 = v2 * v2;
    const auto x  = m_target_distance_meters;
    const auto x2 = x * x;
    const auto y  = m_target_height_meters;

    const auto gx  = g * x;
    const auto gx2 = g * x2;
    const auto v2y = v2 * y;

    const auto numerator = v2 - sqrt(v4 - (g * (gx2 + (2*v2y))));
    const auto denominator = gx;
    const auto θ = atan(numerator / denominator);

    return RAD_TO_DEG(θ);
}

bool t34::TrajMath::IsInRange()
{
    double rate = (
        tan(m_shooter_angle_degrees) - 
        (g * m_target_distance_meters) / (pow(m_shooter_angle_degrees, 2) * pow( (m_note_max_velocity_mps * m_motor_output), 2))
    );

    return rate > 0.0;
}

double t34::TrajMath::GetDistanceFromTarget()
{
    return (
        //(m_target_height_meters - m_limelight_height_meters) / tan( DEG_TO_RAD(m_limelight_angle_degrees) )
        (m_apriltag_height_meters - m_limelight_height_meters) / tan( DEG_TO_RAD(m_target_ty) )
    );
}