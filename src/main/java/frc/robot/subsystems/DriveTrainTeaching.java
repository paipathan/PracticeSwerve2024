// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainTeaching extends SubsystemBase {

  public WPI_TalonSRX frontLeft; // 1
  public WPI_TalonSRX backLeft; // 2
  public WPI_TalonSRX frontRight; // 3
  public WPI_TalonSRX backRight; // 4

  public MotorControllerGroup leftGroup;
  public MotorControllerGroup rightGroup;

  public DifferentialDrive ddrive;
  
  public XboxController controller;

  public DriveTrainTeaching() {

    frontLeft = new WPI_TalonSRX(Constants.MOTOR_L1_ID);
    backLeft = new WPI_TalonSRX(Constants.MOTOR_L2_ID);
    frontRight = new WPI_TalonSRX(Constants.MOTOR_R1_ID);
    backRight = new WPI_TalonSRX(Constants.MOTOR_R2_ID);

    controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);

    leftGroup = new MotorControllerGroup(frontLeft, backLeft);
    rightGroup = new MotorControllerGroup(frontRight, backRight);

    leftGroup.setInverted(true);
    
    ddrive = new DifferentialDrive(leftGroup, rightGroup); // super awesome amazing line

  }

  public void drive() {

    ddrive.arcadeDrive(controller.getLeftY(), controller.getRightX());
    
  }

  @Override
  public void periodic() {


  }
}
