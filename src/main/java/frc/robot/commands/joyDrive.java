// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.annotation.Target;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class joyDrive extends Command {
  private final Drivetrain drivetrain;
  private XboxController controller;
  private final Timer m_timer = new Timer();
  private boolean timerShoot = false;
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
    drivetrain.move(-controller.getLeftY(), controller.getRightX());
    resetEncoderPosition();
    

if (controller.getLeftTriggerAxis() > 0.05) {
    drivetrain.shooter1.set(ControlMode.PercentOutput, -0.5);
      drivetrain.shooter2.set(ControlMode.PercentOutput, -0.5);
}
else {
if (controller.getRightTriggerAxis() > 0.05) {
  drivetrain.shooter1.set(ControlMode.PercentOutput, controller.getRightTriggerAxis());
    drivetrain.shooter2.set(ControlMode.PercentOutput, controller.getRightTriggerAxis());

}
else {
    drivetrain.shooter1.set(ControlMode.PercentOutput, 0);
  drivetrain.shooter2.set(ControlMode.PercentOutput, 0);

}

}
   
    if (controller.getRightBumper()) {
      timerShoot = false;
      drivetrain.shooter1.set(ControlMode.PercentOutput, 0);
      drivetrain.shooter2.set(ControlMode.PercentOutput, 0);
    }

  }

  public void shootNote1() {
    if(controller.getYButton()) {
      drivetrain.shooter1.set(ControlMode.PercentOutput, 1);
    }
    else {
            drivetrain.shooter1.set(ControlMode.PercentOutput, 0);

    }
  }

  public void intakeNote1() {
    if(controller.getXButton()) {
      drivetrain.shooter1.set(ControlMode.PercentOutput, -0.5);
    }
    else {
            drivetrain.shooter1.set(ControlMode.PercentOutput, 0);

    }
  }

   public void shootNote2() {
    if(controller.getBButton()) {
      drivetrain.shooter2.set(ControlMode.PercentOutput, 1);
    }
    else {
            drivetrain.shooter2.set(ControlMode.PercentOutput, 0);

    }
  }

  public void intakeNote2() {
    if(controller.getAButton()) {
      drivetrain.shooter2.set(ControlMode.PercentOutput, -0.5);
    }
    else {
            drivetrain.shooter2.set(ControlMode.PercentOutput, 0);

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
