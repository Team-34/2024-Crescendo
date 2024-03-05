// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Robot.h"

#include "Gyro.h"


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

    rc->shooter.Periodic();
    rc->climber.Periodic();
    rc->limelight_util.m_math_handler.Periodic();
    rc->limelight_util.Periodic();

    frc::SmartDashboard::PutNumber("Distance from limelight target (meters): ", rc->limelight_util.m_math_handler.GetDistanceFromTarget());
    frc::SmartDashboard::PutNumber("Auto drive x: ", rc->limelight_util.m_swerve_drive_speeds[0]);
    frc::SmartDashboard::PutNumber("Auto drive y: ", rc->limelight_util.m_swerve_drive_speeds[1]);
    frc::SmartDashboard::PutNumber("Auto drive r: ", rc->limelight_util.m_swerve_drive_speeds[2]);
    frc::SmartDashboard::PutNumber("Arm firing angle (degrees): ", rc->limelight_util.m_math_handler.GetFiringAngleDeg());
    frc::SmartDashboard::PutNumber("Target ID: ", rc->limelight_util.GetTargetID());
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
        rc->arm_angle_setpoint = ((rc->shooter.GetTopArmEncoderVal() + rc->shooter.GetBottomArmEncoderVal()) * 0.5) / ARM_DEG_SCALAR;
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
        case (0): // amp - up
            rc->shooter.SetMaxSpeedPercent(0.1);
            rc->limelight_util.SetTargetMode(t34::LimelightUtil::TargetMode::kAmp);
            rc->arm_angle_setpoint = 87.18;
            break;
        case (90): // speaker - right
            rc->shooter.SetMaxSpeedPercent(0.4);
            rc->limelight_util.SetTargetMode(t34::LimelightUtil::TargetMode::kSpeaker);
            rc->shooter.MoveToAngleDeg(rc->limelight_util.m_math_handler.GetFiringAngleDeg());
            break;
        case (180): // needs data - down
            rc->shooter.SetMaxSpeedPercent(0.7);
            rc->limelight_util.SetTargetMode(t34::LimelightUtil::TargetMode::kTrap);
            break;
        case (270): // collection mode - left
            rc->shooter.SetMaxSpeedPercent(0.0);
            rc->arm_angle_setpoint = 10.0;
            break;
    }

    //Move the arm with the bumpers
        //Right bumper increases angle, left bumper decreases angle
    if (rc->ctrl->GetLeftBumper() && rc->shooter.UsingPIDArmMovement())
    {
        rc->arm_angle_setpoint -= 1.0;
    }
    else if (rc->ctrl->GetLeftBumper() && rc->shooter.UsingPIDArmMovement() == false)
    {
        rc->shooter.RunTopArmMotorPercent(-0.4);
        rc->shooter.RunBottomArmMotorPercent(-0.4);
    }
    else if (rc->ctrl->GetRightBumper() && rc->shooter.UsingPIDArmMovement())
    {
        rc->arm_angle_setpoint += 1.0;
    }
    else if (rc->ctrl->GetRightBumper() && rc->shooter.UsingPIDArmMovement() == false)
    {
        rc->shooter.RunTopArmMotorPercent(0.4);
        rc->shooter.RunBottomArmMotorPercent(0.4);
    }

    if (rc->shooter.UsingPIDArmMovement())
    {
        rc->shooter.MoveToAngleDeg(std::clamp(rc->arm_angle_setpoint, 0.0, 180.0));
    }
    else
    {
        rc->shooter.RunTopArmMotorPercent(0.0);
        rc->shooter.RunBottomArmMotorPercent(0.0);
    }

    //Run intake with the X button
    if (rc->ctrl->GetXButton())
    {
        rc->shooter.RunIntakeMotorPercent(0.4);
    }
    else
    {
        rc->shooter.RunIntakeMotorPercent(0.0);
    }

    if (rc->ctrl->GetYButton()) // run swerve automatically using the limelight with the Y button
    {
        rc->swerve_drive->Drive
        (
            frc::Translation2d(
                units::meter_t(rc->limelight_util.m_swerve_drive_speeds[0]),
                units::meter_t(rc->limelight_util.m_swerve_drive_speeds[1])
            ),
            rc->limelight_util.m_swerve_drive_speeds[2]
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
