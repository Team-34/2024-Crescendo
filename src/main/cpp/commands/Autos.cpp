// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"
#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>

#include "commands/ExampleCommand.h"


//Commands

//Ptr returning

//frc2::CommandPtr autos::IntakeNote(RobotContainer *m_rc) 
//{
//  return frc2::InstantCommand
//  (
//    [m_rc]
//    {
//      m_rc->shooter.RunIntakeMotorPercent(0.5);
//    }
//  );
//}

frc2::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return frc2::cmd::Sequence(subsystem->ExampleMethodCommand(),
                             ExampleCommand(subsystem).ToPtr());
}
