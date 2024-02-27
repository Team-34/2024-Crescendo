#include "TrajMath.h"

double t34::TrajMath::GetFiringAngle()
{
    //                   ____________________
    //          ⎛  v² - √ v⁴ - g(gx² + 2v²y)   ⎞
    // θ = atan ⎜ ———————————————————————————— ⎟
    //          ⎝               gx             ⎠
    //
    // Source:
    //   Solving Ballistic Trajectories <https://www.forrestthewoods.com/blog/solving_ballistic_trajectories/>
    //   See section “Firing Angle to Hit Stationary Target.”

    const auto v² = m_note_max_velocity_mps * m_note_max_velocity_mps;
    const auto v⁴ = v² * v²;
    const auto x = m_target_distance_meters;
    const auto x² = x * x;
    const auto y = m_target_height_meters;
    const auto g = 9.80665; // gravity

    const auto gx² = g * x²;
    const auto v²y = v² * y;
    const auto gx = g * x;

    const auto numerator = v² - sqrt(v⁴ - (g * (gx² + (2  * v²y))));
    const auto denominator = gx;
    const auto θ = atan(numerator / denominator);

    return RAD_TO_DEG(θ);
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
