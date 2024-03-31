#include "commands/ScoreSpeakerSetup.h"

t34::ScoreSpeakerSetup::ScoreSpeakerSetup(
    t34::Shooter* shooter,
    t34::LimelightUtil &limelight,
    t34::TrajMath &trajectory
) : m_shooter(shooter), m_limelight(limelight), m_trajectory(trajectory)
{
    this->AddRequirements(m_shooter);
}

void t34::ScoreSpeakerSetup::Initialize()
{
    m_limelight.TargetSpeaker();
}

void t34::ScoreSpeakerSetup::Execute()
{
    if (m_limelight.IsTargetAcquired() && m_trajectory.IsInRange())
    {
        m_shooter->MoveToShooterAngleDeg(m_trajectory.GetFiringAngleDeg());
        m_shooter->RunShooterPercent(0.7);
    }
}

bool t34::ScoreSpeakerSetup::IsFinished()
{
    return m_limelight.IsTargetAcquired() 
        && m_trajectory.IsInRange()
        && m_shooter->IsShooterAt(m_trajectory.GetFiringAngle());
}
