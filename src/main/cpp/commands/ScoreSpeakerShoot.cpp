#include "commands/ScoreSpeakerShoot.h"

t34::ScoreSpeakerShoot::ScoreSpeakerShoot(
    t34::Shooter* shooter,
    t34::LimelightUtil &limelight,
    t34::TrajMath &trajectory
) : m_shooter(shooter), m_limelight(limelight), m_trajectory(trajectory)
{
    this->AddRequirements(m_shooter);
}

void t34::ScoreSpeakerShoot::Initialize()
{
    m_shooter->SetMaxSpeedForSpeaker();

    m_intake_saw_note = m_intake_sees_note = m_shooter->IntakeSeesNote();
}

void t34::ScoreSpeakerShoot::Execute()
{
    m_shooter->RunShooterPercent(0.7);

    m_intake_saw_note = m_intake_sees_note;
    m_intake_sees_note = m_shooter->IntakeSeesNote();
}

void t34::ScoreSpeakerShoot::End(bool interrupted)
{
    m_shooter->StopShooter();
}

bool t34::ScoreSpeakerShoot::IsFinished()
{
    bool intake_no_longer_sees_note = m_intake_saw_note && !m_intake_sees_note;

    return intake_no_longer_sees_note;
}
