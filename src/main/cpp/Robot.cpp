// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "Robot.h"

#include "Gyro.h"


#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {
    m_rc = RobotContainer::Get();
    frc2::CommandScheduler::GetInstance().SetDefaultCommand(m_rc->SwerveDrive.get(), std::move(m_rc->DefaultCommand));

    // m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    // m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    // frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

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
    static std::shared_ptr<t34::SwerveDrive> drive = m_rc->SwerveDrive;
    static t34::Gyro* gyro = t34::Gyro::Get();
    frc2::CommandScheduler::GetInstance().Run();
    
    frc::SmartDashboard::PutNumber("_Yaw", gyro->GetAngle());
    drive->PutTelemetry();
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

    m_autonomous_command = m_rc->GetAutonomousCommand();

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
    static std::shared_ptr<t34::SwerveDrive> drive = m_rc->SwerveDrive;
    static std::shared_ptr<t34::T34XboxController> drive_controller = m_rc->DriveController;

    // PROCESS CONTROLLER BUTTONS
    // Buttons are implemented this way out of simplicity.
    // Consider using button trigger events with commands instead.

    // Assign Back Button to Faris Mode.
    if (drive_controller->GetBackButtonReleased()) {
        drive->ToggleFarisMode();
    }

    // Assign Start Button to Zeroing Yaw.
    // Note: This is for emergency use only!
    // The robot should be oriented with the front pointed 
    // at the opposite end of the field and sides as 
    // parallel as possible to the fields sides when this
    // button is pressed/released.
    if (drive_controller->GetStartButtonReleased()) {
        gyro->ZeroYaw();
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
