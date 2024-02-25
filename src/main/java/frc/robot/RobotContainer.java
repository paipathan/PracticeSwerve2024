// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.annotation.JsonPropertyDescription;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.joyDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private final XboxController controller;
  //---------- Subsystems ---------//

  public Swerve swerve;
  public Drivetrain drive;

  //---------- COMMANDS -----------//

  public joyDrive joyDrive; 
  public SwerveDrive swerveDrive; 
  public RunIntake runIntake;

  public RobotContainer() {
    drive = new Drivetrain();
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    joyDrive = new joyDrive(drive, controller);
  }

  public Command getAutoCommand() {
    return new PathPlannerAuto("straightauto");
  }
  
  public Command[] getTeleCommands() {
    Command[] commands = new Command[] {swerveDrive};

    return commands;
  }
}