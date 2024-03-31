#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Shooter.h"
#include "subsystems/LimeLightUtil.h"
#include "TrajMath.h"


namespace t34
{
    class ScoreAmpShoot : public frc2::CommandHelper<frc2::Command, ScoreAmpShoot>
    {
    public:
        explicit ScoreAmpShoot(t34::Shooter* shooter, t34::LimelightUtil &limelight, t34::TrajMath &trajectory);

        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;

    private:
        t34::Shooter* m_shooter;
        t34::LimelightUtil &m_limelight;
        t34::TrajMath &m_trajectory;

        /** Did the intake see the note last time through the loop? */
        bool m_intake_saw_note;

        /** Does the intake see the note this time through the loop? */
        bool m_intake_sees_note;
    };
}