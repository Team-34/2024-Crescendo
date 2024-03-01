// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "T34Controller.hpp"
#include "subsystems/ExampleSubsystem.h"
#include "subsystems/SwerveDrive.h"
#include "commands/ControllerDriveCommand.h"

#include <frc/XboxController.h> 

#include <memory>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>


class RobotContainer {
public: // PROPERTIES
    std::shared_ptr<t34::T34XboxController> DriveController;
    std::shared_ptr<t34::SwerveDrive>  SwerveDrive;


    t34::ControllerDriveCommand DefaultCommand;
    //frc2::Command AutonomousCommand;

public: // METHODS
    static RobotContainer* Get();
    frc2::CommandPtr GetAutonomousCommand();

private: // DATA
    ExampleSubsystem m_subsystem;
//    frc::SendableChooser<Command> m_auto_chooser;
    
private: // METHODS
    RobotContainer();
    void ConfigureBindings();
};
