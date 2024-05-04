#include "RobotContainer.h"


#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/time.h>
#include "commands/Autos.h"
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
    : ctrl(new t34::T34XboxController(0))
    , swerve_drive(new t34::SwerveDrive())
    , shooter()
    , traj_math
        (
            (11.884 * 2), //14.062,
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
    , arm_angle_setpoint(90.0)
    , start_score(false)
    , dir(SwerveDirections::kFwd)
    , DefaultCommand(swerve_drive, ctrl) {
    
    ctrl->SetAllAxisDeadband(0.2);

    //NamedCommands::registerCommand("ShootAmp", std::move(autos::IntakeNote(this))); 
    // <- This example method returns CommandPtr
    //NamedCommands::registerCommand("exampleCommand", std::move(/* put corrosponding function here */)); // <- This example method returns CommandPtr
    //NamedCommands::registerCommand("someOtherCommand", std::move(/* put corrosponding function here */.ToPtr()));
    //NamedCommands::registerCommand("someOtherCommandShared", std::make_shared<frc2::/* put some sort of command here */>());

    ConfigureBindings();

    mode_chooser.AddOption("Starting pos: shoot directly front of sub", "Starting pos: shoot directly front of sub");
    mode_chooser.AddOption("Starting pos: drive straight", "Starting pos: drive straight");
    mode_chooser.AddOption("Starting pos: close left of sub", "Starting pos: close left of sub");
    mode_chooser.AddOption("Starting pos: close right of sub", "Starting pos: close right of sub");
    mode_chooser.AddOption("Starting pos: far right of sub", "Starting pos: far right of sub");

    //path_chooser.SetDefaultOption("None", "None");
    //path_chooser.AddOption("Faris room path", "TestingPathFarisRoom");
//
    //path_chooser.AddOption("Start at left, score 2 points", "Left_Score1_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at left, score 3 points", "Left_Score2_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at left, score 4 points", "Left_Score3_NoAmp_MoveOut");
//
    //path_chooser.AddOption("Start at mid, score 2 points", "Mid_Score1_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at mid, score 3 points", "Mid_Score2_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at mid, score 4 points", "Mid_Score3_NoAmp_MoveOut");
//
    //path_chooser.AddOption("Start at right, score 2 points", "Right_Score1_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at right, score 3 points", "Right_Score2_NoAmp_MoveOut");
    //path_chooser.AddOption("Start at right, score 4 points", "Right_Score3_NoAmp_MoveOut");
//
    //path_chooser.AddOption("Start at left, score in amp", "Left_Score0_YesAmp_MoveAuto");
    //path_chooser.AddOption("Start at mid, score in amp", "Middle_Score0_YesAmp_MoveAuto");
    //path_chooser.AddOption("Start at left, score in amp", "Right_Score0_YesAmp_MoveAuto");
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    return frc2::InstantCommand
    (
        [this]
        {
            if (dir == SwerveDirections::kFwd && mode_chooser.GetSelected() == "Starting pos: shoot directly front of sub")
            {
                shooter.SetSetpoint(29.0);
                shooter.SetMaxSpeedPercent(1.0);
                shooter.Shoot(1.0);
            }

            // if (dir == SwerveDirections::kFwd && mode_chooser.GetSelected() == "Starting pos: drive straight")
            // {
            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(0.0),
            //     units::meter_t(-1.0)},
            //     0.0);
            // }

            // if (dir == SwerveDirections::kFwd)
            // {
            //     vert_distance_inch = swerve_drive->GetModulePositions()[0].distance();

            //     if (vert_distance_inch <= -95 && mode_chooser.GetSelected() == "Starting pos: close right of sub")
            //     {
            //         dir = SwerveDirections::kLeft;
            //     }
            //     else if (vert_distance_inch <= -95 && mode_chooser.GetSelected() == "Starting pos: close left of sub")
            //     {
            //         dir = SwerveDirections::kRight;
            //     }

            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(0.0),
            //     units::meter_t(-1.0)},
            //     0.0);
            // }

            // else if (dir == SwerveDirections::kLeft)
            // {
            //     hori_distance_inch = signbit(swerve_drive->GetModulePositions()[0].distance()) ?
            //         swerve_drive->GetModulePositions()[0].distance() + vert_distance_inch :
            //         swerve_drive->GetModulePositions()[0].distance() - vert_distance_inch;

            //     if (hori_distance_inch >= 38 && mode_chooser.GetSelected() == "Starting pos: close right of sub")
            //     {
            //         dir = SwerveDirections::kStill;
            //     }

            //     if (hori_distance_inch >= 118 && mode_chooser.GetSelected() == "Starting pos: far right of sub")
            //     {
            //         dir = SwerveDirections::kStill;
            //     }

            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(-1.0),
            //     units::meter_t(0.0)},
            //     0.0);
            // }

            // else if (dir == SwerveDirections::kRight)
            // {
            //     hori_distance_inch = signbit(swerve_drive->GetModulePositions()[0].distance()) ?
            //         swerve_drive->GetModulePositions()[0].distance() + vert_distance_inch :
            //         swerve_drive->GetModulePositions()[0].distance() - vert_distance_inch;

            //     if (hori_distance_inch <= -38 && mode_chooser.GetSelected() == "Starting pos: close left of sub")
            //     {
            //         dir = SwerveDirections::kStill;
            //     }

            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(1.0),
            //     units::meter_t(0.0)},
            //     0.0);
            // }

            // else if (dir == SwerveDirections::kStill)
            // {
            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(0.0),
            //     units::meter_t(0.0)},
            //     0.0);

            //     shooter.ConfigForSpeaker(traj_math.GetArmFiringAngleDeg());
            //     shooter.Shoot(1.0);
            // }
            // else
            // {
            //     swerve_drive->Drive(frc::Translation2d{
            //     units::meter_t(0.0),
            //     units::meter_t(0.0)},
            //     0.0);
            // }
            
        },
        frc2::Requirements( {swerve_drive.get()} )
    ).ToPtr();

    // const std::string path_file_name = path_chooser.GetSelected();
    // const auto path = pathplanner::PathPlannerPath::fromPathFile(path_file_name);

    // return pathplanner::AutoBuilder::followPath(path);
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

    m_controller.Back().Debounce(100_ms).OnFalse(frc2::cmd::RunOnce(
        [this] { swerve_drive->ToggleFarisMode(); },
        {swerve_drive.get()}
    ));
}
