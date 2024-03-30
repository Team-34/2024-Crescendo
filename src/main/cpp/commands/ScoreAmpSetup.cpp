#include "commands/ScoreAmpSetup.h"
// #include <units/angle.h>
#include <units/math.h>

t34::ScoreAmpSetup::ScoreAmpSetup(
    t34::Shooter* shooter,
    t34::LimelightUtil &limelight,
    t34::TrajMath &trajectory
) : m_shooter(shooter), m_limelight(limelight), m_trajectory(trajectory)
{
    this->AddRequirements(m_shooter);
}

void t34::ScoreAmpSetup::Initialize()
{
    m_shooter->SetMaxSpeedForAmp();
    m_limelight.TargetAmp();
}

void t34::ScoreAmpSetup::Execute()
{
    if (m_limelight.IsTargetAcquired() && m_trajectory.IsInRange())
    {
        m_shooter->MoveToShooterAngleDeg(m_trajectory.GetFiringAngleDeg());
    }
}

void t34::ScoreAmpSetup::End(bool interrupted)
{
    // do nothing
}

bool t34::ScoreAmpSetup::IsFinished()
{
    if (m_limelight.IsTargetAcquired() && m_trajectory.IsInRange())
    {
        const auto THRESHOLD = 0.5_deg;
        const auto delta = m_shooter->GetShooterAngle() - m_trajectory.GetFiringAngle();
        return units::math::abs(delta) < THRESHOLD;
    }

    return false;
}
