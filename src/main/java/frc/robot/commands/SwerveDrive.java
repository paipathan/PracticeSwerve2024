// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.S_Drive;

public class SwerveDrive extends Command {

  public S_Drive s_drive;
  public XboxController controller;

  public SwerveDrive() {
    s_drive = new S_Drive();
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_drive.drive();
    s_drive.resetPostion();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
