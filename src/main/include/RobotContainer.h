// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "T34Controller.hpp"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/SwerveDrive.h"
#include "commands/ControllerDriveCommand.h"
#include "Constants.h"
#include <iostream>

#include <pathplanner/lib/commands/PathPlannerAuto.h>

#include <memory>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "subsystems/SwerveDrive.h"
#include "subsystems/Shooter.h"
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

using namespace pathplanner;
using namespace t34;

class RobotContainer {
public: // PROPERTIES
    std::shared_ptr<t34::T34XboxController> DriveController;
    std::shared_ptr<t34::SwerveDrive>  SwerveDrive;
    std::shared_ptr<t34::Shooter> m_shooter;


    t34::ControllerDriveCommand DefaultCommand;

   
    //frc2::Command AutonomousCommand;
public: // METHODS

    static RobotContainer* Get();
    frc2::CommandPtr GetAutonomousCommand();

    

private: // DATA
    ExampleSubsystem m_subsystem;
    frc::SendableChooser<std::string> m_chooser;

    // frc2::CommandPtr m_exampleSelectCommand = frc2::cmd::Select<std::string>(
    //   [this] { return m_chooser.GetSelected(); },
    //   // Maps selector values to commands
    //   m_chooser.AddOption("Auto Selection One", )

    //frc::SendableChooser<Command> m_auto_chooser;
    
private: // METHODS
    RobotContainer();
    void ConfigureBindings();
};
