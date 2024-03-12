// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Robot.h"

#include "Gyro.h"
#include <cmath>
#include "LimelightHelpers.h"
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
    rc = RobotContainer::Get();
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(rc->swerve_drive.get(), std::move(rc->DefaultCommand));

    rc->shooter.Init();
    rc->climber.Init();
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
    //static std::shared_ptr<t34::SwerveDrive> drive = m_rc->SwerveDrive;
    static t34::Gyro* gyro = t34::Gyro::Get();
    frc2::CommandScheduler::GetInstance().Run();
    
    frc::SmartDashboard::PutNumber("_Yaw", gyro->GetAngle());

    rc->swerve_drive->PutTelemetry();
    rc->shooter.PutTelemetry();

    //Periodics
    rc->shooter.Periodic();
    rc->climber.Periodic();
    rc->limelight_util.Periodic();
    //_________________________

    // Swerve Auto Drive Outputs
    frc::SmartDashboard::PutNumber("Auto drive x: ", rc->limelight_util.m_swerve_drive_speeds.x);
    frc::SmartDashboard::PutNumber("Auto drive y: ", rc->limelight_util.m_swerve_drive_speeds.y);
    frc::SmartDashboard::PutNumber("Auto drive r: ", rc->limelight_util.m_swerve_drive_speeds.r);
    //_________________________


    // Misc.
    frc::SmartDashboard::PutNumber("Target ID: ", rc->limelight_util.GetTargetID());
    frc::SmartDashboard::PutNumber("Distance from limelight target (meters): ", rc->limelight_util.m_math_handler.GetDistanceFromTarget());
    //_________________________
    /*
    //checks if the approx. range is nearing 5 meters (the range the LL with pick up an AT before the resolution becomes too low)
    if (log2( (2.5 / LimelightHelpers::getTA()) - 0.3) >= 5.0) {
        frc::SmartDashboard::PutNumber("Over maximum detection range, current distance is: ", rc->limelight_util.m_math_handler.GetDistanceFromTarget());
    }
    //_________________________*/
    
    rc->limelight_util.m_math_handler.PutTelemetry();
    
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}



/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {

    m_autonomous_command = rc->GetAutonomousCommand();

    if (m_autonomous_command)
    {
        m_autonomous_command->Schedule();
    }

    // m_autoSelected = m_chooser.GetSelected();
    // frc::SmartDashboard::PutString("Auto selected: ", m_autoSelected);

    // if (m_autoSelected == kAutoNameCustom) {
    //     // Custom Auto goes here
    // } else {
    //     // Default Auto goes here
    // }

}

void Robot::AutonomousPeriodic()
{

}

void Robot::TeleopInit() {

      // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomous_command)
  {
    m_autonomous_command->Cancel();
    m_autonomous_command.reset();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    static t34::Gyro* gyro = t34::Gyro::Get();
    //static std::shared_ptr<t34::SwerveDrive> drive = m_rc->swerve_drive;
    //static std::shared_ptr<t34::T34XboxController> drive_controller = m_rc->ctrl;
    
    rc->limelight_util.m_math_handler.InputMotorOutputPercent(rc->shooter.GetMaxSpeedPercent());

    // PROCESS CONTROLLER BUTTONS
    // Buttons are implemented this way out of simplicity.
    // Consider using button trigger events with commands instead.

    // Assign Back Button to Faris Mode.
    if (rc->ctrl->GetBackButtonReleased())
    {
        rc->swerve_drive->ToggleFarisMode();
    }

    // Assign Start Button to Zeroing Yaw.
    // Note: This is for emergency use only!
    // The robot should be oriented with the front pointed 
    // at the opposite end of the field and sides as 
    // parallel as possible to the fields sides when this
    // button is pressed/released.
    if (rc->ctrl->GetStartButtonReleased()) {
        gyro->ZeroYaw();
    }

    // toggle PID vs basic motor output arm movement with the A button
    if (rc->ctrl->GetAButtonReleased()) { 
        rc->shooter.TogglePIDArmMovement();
    }

    //Run the shooter with the triggers
        //Right is forward, left is back
    if (rc->ctrl->GetLeftTriggerAxis() > 0.2)
    {
        rc->shooter.RunShooterPercent(-(rc->ctrl->GetLeftTriggerAxis()));
    }
    else if (rc->ctrl->GetRightTriggerAxis() > 0.2)
    {
        rc->shooter.RunShooterPercent(rc->ctrl->GetRightTriggerAxis());
    }
    else
    {
        rc->shooter.RunShooterPercent(0.0);
    }

    //Set the robot's target mode with the D-Pad
    switch (rc->ctrl->GetPOV())
    {
        case (POV_AMP): // amp - up
            rc->limelight_util.TargetAmp();
            rc->shooter.TargetAmp();
            break;
        case (POV_SPEAKER): // speaker - right
            rc->limelight_util.TargetSpeaker();
            rc->shooter.TargetSpeaker(rc->limelight_util.m_math_handler.GetFiringAngleDeg());
            break;
        case (POV_TRAP): // needs data - down
            rc->limelight_util.TargetTrap();
            rc->shooter.TargetTrap();
            break;
        case (POV_COLLECTION): // collection mode - left
            rc->shooter.CollectNotes();
            break;
    }

    //Move the arm with the bumpers
        //Right bumper increases angle, left bumper decreases angle
    if (rc->ctrl->GetLeftBumper())
    {
        rc->shooter.LowerArm();
    }
    else if (rc->ctrl->GetRightBumper())
    {
        rc->shooter.RaiseArm();
    }
    else
    {
        rc->shooter.StopArm();
    }

    rc->shooter.Periodic();

    //Run intake with the X button
    if (rc->ctrl->GetXButton())
    {
        rc->shooter.RunIntake();
    }
    else
    {
        rc->shooter.StopIntake();
    }

    if (rc->ctrl->GetYButton()) // run swerve automatically using the limelight with the Y button
    {
        rc->swerve_drive->Drive
        (
            frc::Translation2d(
                units::meter_t(rc->limelight_util.m_swerve_drive_speeds.x), 
                units::meter_t(rc->limelight_util.m_swerve_drive_speeds.y)
            ),
            rc->limelight_util.m_swerve_drive_speeds.r
        );
    }
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
  int main() {
      return frc::StartRobot<Robot>();
  }
#endif
