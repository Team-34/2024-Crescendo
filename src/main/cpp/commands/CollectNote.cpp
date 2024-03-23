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
    
}

void t34::CollectNote::End(bool interrupted)
{

}

bool t34::CollectNote::IsFinished()
{

}
