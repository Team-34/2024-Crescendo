#include "commands/ScoreAmpShoot.h"

t34::ScoreAmpShoot::ScoreAmpShoot(t34::Shooter* shooter, t34::LimelightUtil &limelight, t34::TrajMath &trajectory)
    : m_shooter(shooter), m_limelight(limelight), m_trajectory(trajectory)
{
    this->AddRequirements(m_shooter);
}

void t34::ScoreAmpShoot::Initialize()
{
    m_shooter->SetMaxSpeedForAmp();

    m_intake_sees_note_previous = m_shooter->IntakeSeesNote();
    m_intake_sees_note_current = m_shooter->IntakeSeesNote();
}

void t34::ScoreAmpShoot::Execute()
{
    m_shooter->RunShooterPercent(0.5);

    m_intake_sees_note_previous = m_intake_sees_note_current;
    m_intake_sees_note_current = m_shooter->IntakeSeesNote();
}

void t34::ScoreAmpShoot::End(bool interrupted)
{
    m_shooter->RunShooterPercent(0.0);
}

bool t34::ScoreAmpShoot::IsFinished()
{
    bool intake_no_longer_sees_note = m_intake_sees_note_previous && !m_intake_sees_note_current;
    
    return intake_no_longer_sees_note;
}
