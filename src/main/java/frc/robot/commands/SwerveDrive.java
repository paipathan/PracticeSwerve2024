// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.S_Drive;
import frc.robot.subsystems.Swerve;

public class SwerveDrive extends Command {

  public Swerve swerve;
  public S_Drive s_drive;
  public XboxController controller;

  public SwerveDrive() {
    swerve = new Swerve();
    s_drive = new S_Drive();
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // swerve.driveWithJoystick(controller); // WPILIB swerve mods 
    s_drive.drive(); // custom swerve mods
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
