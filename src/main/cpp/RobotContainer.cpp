#include "RobotContainer.h"


#include <frc2/command/button/Trigger.h>

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
    : swerve_drive(new t34::SwerveDrive())
    , ctrl(new t34::T34XboxController(0))
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
    , auto_start_dist_1(0.0)
    , auto_end_dist_1(0.0)
    , auto_start_dist_2(0.0)
    , auto_end_dist_2(0.0)
    , auto_finished_driving_1(false)
    , auto_finished_aiming(true)
    , auto_finished_driving_2(true)
    , auto_finished_shooting(true)
    , DefaultCommand(swerve_drive, ctrl) {
    
    ctrl->SetAllAxisDeadband(0.2);

    //NamedCommands::registerCommand("ShootAmp", std::move(autos::IntakeNote(this))); 
    // <- This example method returns CommandPtr
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

    path_chooser.AddOption("Start at left, score in amp", "Left_Score0_YesAmp_MoveAuto");
    path_chooser.AddOption("Start at mid, score in amp", "Middle_Score0_YesAmp_MoveAuto");
    path_chooser.AddOption("Start at left, score in amp", "Right_Score0_YesAmp_MoveAuto");
}


frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    //this->auto_start_dist_1 = 0.0;
    //this->auto_end_dist_1 = 80.15;
//
    //this->auto_start_dist_2 = 0.0;
    //this->auto_end_dist_2 = 74.0;
//
    //this->auto_current_dist = this->swerve_drive->GetModulePositions()[0].distance();

    return frc2::InstantCommand
    (
        [this]
        {
            //this->auto_current_dist = this->swerve_drive->GetModulePositions()[0].distance();
//
//
            //if (this->auto_current_dist < this->auto_end_dist_1 && this->auto_finished_driving_1 == false)
            //{
                this->swerve_drive->Drive(frc::Translation2d{
                units::meter_t(0.0),
                units::meter_t(-1.0)},//std::copysign(ScaleToRange(-(345.0 * 345.0), 0.0, 1.0, 0.0, 0.4), 345.0))),
                0.0);
            //}
            //else
            //{
            //    this->auto_finished_driving_1 = true;
            //    this->auto_finished_driving_2 = false;
////
            //    this->auto_end_dist_2 += this->auto_current_dist;
            //}
    //////////////////////
            //if (this->auto_current_dist < this->auto_end_dist_1 && this->auto_finished_driving_2 == false)
            //{
            //    this->swerve_drive->Drive(frc::Translation2d{
            //    units::meter_t(-0.5),
            //    units::meter_t(0.0)},//std::copysign(ScaleToRange(-(345.0 * 345.0), 0.0, 1.0, 0.0, 0.4), 345.0))),
            //    0.0);
            //}
            //else
            //{
            //    this->auto_finished_driving_1 = true;
            //    this->auto_finished_aiming = false;
            //}
    /////////////////////
            //if (this->auto_finished_aiming == false)
            //{
            //    if (this->shooter.GetTopArmEncoderVal() <= (85.0))
            //    {
            //        this->shooter.RunTopArmMotorPercent(0.3);
            //        this->shooter.RunBottomArmMotorPercent(0.3);
            //    }
            //    else if (this->shooter.GetTopArmEncoderVal() >= (89.0))
            //    {
            //        this->shooter.RunTopArmMotorPercent(-0.15);
            //        this->shooter.RunBottomArmMotorPercent(-0.15);
            //    }
            //    else
            //    {
            //        this->shooter.RunTopArmMotorPercent(0.0);
            //        this->shooter.RunBottomArmMotorPercent(0.0);
            //        this->auto_finished_aiming = true;
            //    }
            //}
    //////////////////////
            //if (this->auto_finished_aiming)
            //{
            //    this->shooter.RunShooterPercent(0.1);
            //    this->shooter.RunIntakeMotorPercent(0.6);
            //}
        },
        frc2::Requirements( {swerve_drive.get()} )
    ).ToPtr();

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
