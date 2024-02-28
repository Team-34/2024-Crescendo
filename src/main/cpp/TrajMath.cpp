#include "TrajMath.h"

t34::TrajMath::TrajMath
(
    double note_max_velocity_mps,
    double target_height_meters,
    double limelight_height_meters,
    double shooter_angle,
    double limelight_angle
):
    m_note_max_velocity_mps(note_max_velocity_mps),
    m_target_height_meters(target_height_meters),
    m_limelight_height_meters(limelight_height_meters),
    m_shooter_angle(shooter_angle),
    m_limelight_angle(limelight_angle),
    m_target_tx(0.0),
    m_target_ty(0.0)
{}     

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

    const auto v_2 = m_note_max_velocity_mps * m_note_max_velocity_mps;
    const auto v_4 = v_2 * v_2;
    const auto x = m_target_distance_meters;
    const auto x_2 = x * x;
    const auto y = m_target_height_meters;

    const auto gx = g * x;
    const auto gx_2 = g * x_2;
    const auto yv_2 = y * v_2;

    const auto numerator = v_2 - sqrt(v_4 - (g * (gx_2 + (2  * yv_2))));
    const auto denominator = gx;
    const auto θ = atan(numerator / denominator);

    return RAD_TO_DEG(θ);
}

bool t34::TrajMath::IsInRange()
{
    double rate = (
        tan(m_shooter_angle) - 
        (g * m_target_distance_meters) / (pow(m_shooter_angle, 2) * pow(m_note_max_velocity_mps, 2))
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
