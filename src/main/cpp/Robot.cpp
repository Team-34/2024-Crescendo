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
void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
    static t34::Gyro* gyro = t34::Gyro::Get();
    //static std::shared_ptr<t34::SwerveDrive> drive = m_rc->swerve_drive;
    //static std::shared_ptr<t34::T34XboxController> drive_controller = m_rc->ctrl;

    // PROCESS CONTROLLER BUTTONS
    // Buttons are implemented this way out of simplicity.
    // Consider using button trigger events with commands instead.

    // Assign Back Button to Faris Mode.
    if (rc->ctrl->GetBackButtonReleased()) {
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


    //Run the shooter with the triggers
        //Right is forward, left is back
    if (rc->ctrl->GetLeftTriggerAxis() > 0.2)
    {
        rc->shooter.RunShooter(-(rc->ctrl->GetLeftTriggerAxis()));
    }
    else if (rc->ctrl->GetRightTriggerAxis() > 0.2)
    {
        rc->shooter.RunShooter(rc->ctrl->GetRightTriggerAxis());
    }
    else
    {
        rc->shooter.RunShooter(0.0);
    }

    //Set the shooter's max speed with the D-Pad
    switch (rc->ctrl->GetPOV())
    {
        case (0):
            rc->shooter.SetMaxSpeedPercent(0.1);
            break;
        case (90):
            rc->shooter.SetMaxSpeedPercent(0.4);
            break;
        case (180):
            rc->shooter.SetMaxSpeedPercent(0.7);
            break;
        case (270):
            rc->shooter.SetMaxSpeedPercent(1.0);
            break;
    }

    //Move the arm with the bumpers
        //Right bumper increases angle, left bumper decreases angle
    if (rc->ctrl->GetLeftBumper())
    {
        rc->arm_angle_setpoint -= 1.0;

        rc->shooter.RunTopArmMotor(-0.4);
        rc->shooter.RunBottomArmMotor(-0.4);
    }
    else if (rc->ctrl->GetRightBumper())
    {
        rc->arm_angle_setpoint += 1.0;

        rc->shooter.RunTopArmMotor(0.4);
        rc->shooter.RunBottomArmMotor(0.4);
    }
    else
    {
        rc->shooter.RunTopArmMotor(0.0);
        rc->shooter.RunBottomArmMotor(0.0);
    }

    //rc->shooter.MoveToAngleDeg(std::clamp(rc->arm_angle_setpoint, -170.0, 180.0));

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
