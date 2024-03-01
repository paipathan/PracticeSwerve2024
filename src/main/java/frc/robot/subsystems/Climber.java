// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;




public class Climber extends SubsystemBase {

  public XboxController controller;

  public CANSparkMax motor1;
  public CANSparkMax motor2;



  public Climber() {
    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    motor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_1, MotorType.kBrushless);
    motor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_2, MotorType.kBrushless);
  }


  public void buttonClimb() {
    if(controller.getYButton()) {
      motor1.set(1);
      motor2.set(1);
    } else if(controller.getBButton()) {
      motor1.set(-1);
      motor2.set(-1);
    }

    SmartDashboard.putNumber("m1 volt", motor1.getBusVoltage());
    SmartDashboard.putNumber("m2 volt", motor2.getBusVoltage());
  }

  public void joystickClimb() {
    motor1.set(controller.getLeftX());
    motor2.set(controller.getLeftX());
    SmartDashboard.putNumber("m1 volt", motor1.getBusVoltage());
    SmartDashboard.putNumber("m2 volt", motor2.getBusVoltage());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
