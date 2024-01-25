// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class joyDrive extends Command {
  private final Drivetrain drivetrain;
  private XboxController controller;
  
  public joyDrive(Drivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.move(controller.getLeftY(), controller.getRightX());
    resetEncoderPosition();
    shootNote();
    intakeNote();
  }

  public void shootNote() {
    if(controller.getXButton()) {
      drivetrain.shooter1.set(ControlMode.PercentOutput, 1);
      drivetrain.shooter2.set(ControlMode.PercentOutput, 1);
    }
  }

  public void intakeNote() {
    if(controller.getBButton()) {
      drivetrain.shooter1.set(ControlMode.PercentOutput, -1);
      drivetrain.shooter2.set(ControlMode.PercentOutput, -1);
    }
  }

  public void resetEncoderPosition() {
    if (controller.getAButton()) {
    drivetrain.frontLeft.setSelectedSensorPosition(0);
    drivetrain.frontRight.setSelectedSensorPosition(0);
    }
  }





  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
