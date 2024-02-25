// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  
  public TalonSRX intake;
  public DigitalInput banner;

  public Intake() {
    intake = new TalonSRX(Constants.INTAKE_MOTOR_ID);
    banner = new DigitalInput(Constants.BANNER_SENSOR_PORT);
  }

  public void runIntake(XboxController controller, boolean usingBanner) {

    if(usingBanner) { 
      double power = (controller.getYButton() && !banner.get()) ? -1: 0;
      intake.set(ControlMode.PercentOutput, power);
    } else { 
      double power = (controller.getYButton()) ? -1 : 0;
      intake.set(ControlMode.PercentOutput, power);  
    }

    SmartDashboard.putBoolean("Using Banner?", usingBanner);
    SmartDashboard.putBoolean("Banner Triggered?", banner.get());
    SmartDashboard.putNumber("Motor Output %", intake.getMotorOutputPercent());


  }

  @Override
  public void periodic() {
    
  }

  
}
