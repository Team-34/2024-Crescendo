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
    m_shooter->SetMaxSpeedForSpeaker();
    m_limelight.TargetSpeaker();
}

void t34::ScoreSpeakerSetup::Execute()
{
    if( m_limelight.IsTargetAcquired() && m_trajectory.IsInRange() )
    {
        m_shooter->MoveToShooterAngleDeg(m_trajectory.GetFiringAngleDeg());
        m_shooter->RunShooterPercent(0.7);
    }
}

void t34::ScoreSpeakerSetup::End(bool interrupted)
{
    // m_shooter->RunShooterPercent(0.0);
}

bool t34::ScoreSpeakerSetup::IsFinished()
{
    if (m_limelight.IsTargetAcquired() && m_trajectory.IsInRange())
    {
        const auto THRESHOLD = 0.5_deg;
        const auto delta = m_shooter->GetShooterAngle() - m_trajectory.GetFiringAngle();
        return units::math::abs(delta) < THRESHOLD;
    }

    return false;
}
