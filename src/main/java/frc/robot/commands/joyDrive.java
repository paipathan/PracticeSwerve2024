// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Swerve;


public class joyDrive extends Command {
  
  public XboxController controller;
  public Drivetrain drive;


  public joyDrive(Drivetrain drive, XboxController controller) {
    this.drive = drive;
    this.controller = controller;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    
  }


  @Override
  public void execute() {
    drive.move(-controller.getLeftY(), controller.getRightX());
  }


  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
