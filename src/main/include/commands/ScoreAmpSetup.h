#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"
#include "subsystems/LimeLightUtil.h"
#include "TrajMath.h"


namespace t34
{
    class ScoreAmpSetup : public frc2::CommandHelper<frc2::Command, ScoreAmpSetup>
    {
    public:
        explicit ScoreAmpSetup(t34::Shooter* shooter, t34::LimelightUtil &limelight, t34::TrajMath &trajectory);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        t34::Shooter* m_shooter;
        t34::LimelightUtil &m_limelight;
        t34::TrajMath &m_trajectory;
    };
}
