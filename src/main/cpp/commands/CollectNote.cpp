#include "commands/CollectNote.h"
#include "subsystems/Shooter.h"

t34::CollectNote::CollectNote(t34::Shooter* shooter) : m_shooter(shooter)
{
    this->AddRequirements(m_shooter);
}

void t34::CollectNote::Initialize()
{
    m_shooter->SetMaxSpeedForNoteCollection();
    m_shooter->LowerArmForNoteCollection();
}

void t34::CollectNote::Execute()
{
    m_shooter->RunIntakeMotorPercent(0.5);
}

void t34::CollectNote::End(bool interrupted)
{
    m_shooter->RunIntakeMotorPercent(0.0);
}

bool t34::CollectNote::IsFinished()
{
    return m_shooter->IntakeSeesNote();
}
