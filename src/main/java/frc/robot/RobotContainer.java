// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.joyDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private final XboxController controller;

  private final Drivetrain drivetrain;
  private final joyDrive joyDrive;

  public RobotContainer() {
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    drivetrain = new Drivetrain();
    joyDrive = new joyDrive(drivetrain, controller);
  }

  public Command getAutoCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("new auto");
    return AutoBuilder.followPathWithEvents(path);
  }
  
  public Command[] getTeleCommands() {
    Command[] commands = new Command[] {
      joyDrive
    };

    return commands;
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
    
  }
}