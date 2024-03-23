#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"

namespace t34
{
    class CollectNote : public frc2::CommandHelper<frc2::Command, CollectNote>
    {
    public:
        explicit CollectNote(t34::Shooter* shooter);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        t34::Shooter* m_shooter;
    };
}
