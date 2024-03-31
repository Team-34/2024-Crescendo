#include "commands/ScoreAmpSetup.h"

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

bool t34::ScoreAmpSetup::IsFinished()
{
    return m_limelight.IsTargetAcquired() 
        && m_trajectory.IsInRange()
        && m_shooter->IsShooterAt(m_trajectory.GetFiringAngle());
}
