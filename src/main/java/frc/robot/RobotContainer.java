// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.spi.ResourceBundleProvider;

import javax.swing.text.html.HTMLDocument.RunElement;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RunClimber;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SwerveDrive;
import frc.robot.commands.joyDrive;

public class RobotContainer {



  //---------- COMMANDS -----------//

  public joyDrive joyDrive; 
  public SwerveDrive swerveDrive; 
  public RunIntake runIntake;
  public RunClimber runClimber;

  //-------------------------------//

  public RobotContainer() { // initialize commands
    joyDrive = new joyDrive();
    swerveDrive = new SwerveDrive();
    runIntake = new RunIntake();
    runClimber = new RunClimber();
  }

  public Command getAutoCommand() {
    return new PathPlannerAuto("straightauto");
  }
  
  public Command[] getTeleCommands() { // call commands we want to run
    Command[] commands = new Command[] {runClimber};

    return commands;
  }
}