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
    : swerve_drive(new t34::SwerveDrive())
    , ctrl(new t34::T34XboxController(0))
    , shooter()
    , climber()
    , traj_math
        (
            23.884,
            1.9815,
            1.435,
            0.2688,
            ((shooter.GetTopArmEncoderVal() + shooter.GetBottomArmEncoderVal()) * 0.5) / ARM_DEG_SCALAR,
            30.0
        )
    , limelight_util
        (
            traj_math,
            t34::LimelightUtil::TargetMode::kSpeaker
        )
    , arm_angle_setpoint(0.5)
    , DefaultCommand(swerve_drive, ctrl) {
    
    ctrl->SetAllAxisDeadband(0.2);

    //NamedCommands::registerCommand("ShootSpeaker", std::move(/* put corrosponding function here */)); // <- This example method returns CommandPtr
    //NamedCommands::registerCommand("exampleCommand", std::move(/* put corrosponding function here */)); // <- This example method returns CommandPtr
    //NamedCommands::registerCommand("someOtherCommand", std::move(/* put corrosponding function here */.ToPtr()));
    //NamedCommands::registerCommand("someOtherCommandShared", std::make_shared<frc2::/* put some sort of command here */>());

    ConfigureBindings();

    path_chooser.SetDefaultOption("None", "None");
    path_chooser.AddOption("Faris room path", "TestingPathFarisRoom");

    path_chooser.AddOption("Start at left, score 2 points", "Left_Score1_NoAmp_MoveOut");
    path_chooser.AddOption("Start at left, score 3 points", "Left_Score2_NoAmp_MoveOut");
    path_chooser.AddOption("Start at left, score 4 points", "Left_Score3_NoAmp_MoveOut");

    path_chooser.AddOption("Start at mid, score 2 points", "Mid_Score1_NoAmp_MoveOut");
    path_chooser.AddOption("Start at mid, score 3 points", "Mid_Score2_NoAmp_MoveOut");
    path_chooser.AddOption("Start at mid, score 4 points", "Mid_Score3_NoAmp_MoveOut");

    path_chooser.AddOption("Start at right, score 2 points", "Right_Score1_NoAmp_MoveOut");
    path_chooser.AddOption("Start at right, score 3 points", "Right_Score2_NoAmp_MoveOut");
    path_chooser.AddOption("Start at right, score 4 points", "Right_Score3_NoAmp_MoveOut");
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    const std::string path_file_name = path_chooser.GetSelected();
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
