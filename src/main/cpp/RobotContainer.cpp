#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"
#include "subsystems/Shooter.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>


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
    , DefaultCommand(SwerveDrive, DriveController)
    , m_shooter()
{

    DriveController->SetAllAxisDeadband(0.2);

    // USE THIS LINE OF CODE: // NamedCommands::registerCommand("ShootSpeaker", std::move(/* put corrosponding function here */)); // <- This example method returns CommandPtr
    // USE THIS LINE OF CODE: // NamedCommands::registerCommand("exampleCommand", std::move(/* put corrosponding function here */)); // <- This example method returns CommandPtr
    // USE THIS LINE OF CODE: // NamedCommands::registerCommand("someOtherCommand", std::move(/* put corrosponding function here */.ToPtr()));
    // USE THIS LINE OF CODE: // NamedCommands::registerCommand("someOtherCommandShared", std::make_shared<frc2::/* put some sort of command here */>());

    ConfigureBindings();

    // Build an auto chooser. This will use Commands.none() as the default option.
    // m_auto_chooser = pathplanner::AutoBuilder::b buildAutoChooser();

    m_chooser.SetDefaultOption("Back Up and Shoot", "Back Up and Shoot");
    m_chooser.AddOption("Back Up", "Back Up");
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    const std::string path_file_name = m_chooser.GetSelected();
    const auto path = pathplanner::PathPlannerPath::fromPathFile(path_file_name);
    return pathplanner::AutoBuilder::followPath(path);
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
