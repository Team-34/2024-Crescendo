#pragma once

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/button/Trigger.h>

namespace t34
{
    /**
     * Custom CommandXboxController class that adds Triggers for events that
     * CommandXboxController only defines BooleanEvents for.
     */
    class T34CommandXboxController : public frc2::CommandXboxController
    {
    public:
        T34CommandXboxController(int port) : frc2::CommandXboxController(port)
        {
        }

        T34CommandXboxController(T34CommandXboxController &&other)
        : frc2::CommandXboxController(other.GetPort())
        {
        }

        frc2::Trigger POVDown(frc::EventLoop *loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const
        {
            return frc2::CommandXboxController::POVDown(loop).CastTo<frc2::Trigger>();
        }

        frc2::Trigger POVLeft(frc::EventLoop *loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const
        {
            return frc2::CommandXboxController::POVLeft(loop).CastTo<frc2::Trigger>();
        }

        frc2::Trigger POVRight(frc::EventLoop *loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const
        {
            return frc2::CommandXboxController::POVRight(loop).CastTo<frc2::Trigger>();
        }

        frc2::Trigger POVUp(frc::EventLoop *loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()) const
        {
            return frc2::CommandXboxController::POVUp(loop).CastTo<frc2::Trigger>();
        }
    };
}
