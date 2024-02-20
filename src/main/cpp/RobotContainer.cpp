#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

static std::unique_ptr<RobotContainer> g_rc{ nullptr };

RobotContainer* RobotContainer::Get() {
    if (!g_rc) {
        g_rc.reset(new RobotContainer());
    }

    return g_rc.get();
}


RobotContainer::RobotContainer()
    : SwerveDrive(new t34::SwerveDrive())
    , DriveController(new t34::T34XboxController(0))
    , DefaultCommand(SwerveDrive, DriveController) {

    DriveController->SetAllAxisDeadband(0.2);

    ConfigureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
//    m_auto_chooser = AutoBuilder::buildAutoChooser();
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    // An example command will be run in autonomous
    return autos::ExampleAuto(&m_subsystem);
    //return PathPlannerAuto(m_auto_chooser.GetSelected()).ToPtr();
}

void RobotContainer::ConfigureBindings() {
    // Configure your trigger bindings here

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // frc2::Trigger([this] {
    //     return m_subsystem.ExampleCondition();
    // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

    // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
    // pressed, cancelling on release.
    // DriveController->B().WhileTrue(m_subsystem.ExampleMethodCommand());
}
