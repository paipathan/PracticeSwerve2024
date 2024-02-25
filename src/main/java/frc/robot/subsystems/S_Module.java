// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class S_Module extends SubsystemBase {
  public TalonFX driveMotor;
  public TalonFX steerMotor;
  public CANcoder encoder;
  public TalonFXConfiguration motorConfigs;
  
  public S_Module(int driveId, int steerID, int canID) {
    this.driveMotor = new TalonFX(driveId);
    this.steerMotor = new TalonFX(steerID);
    this.encoder = new CANcoder(canID);
    motorConfigs = new TalonFXConfiguration();

  }

  public void set(double speedMPS, double rot) {
    motorConfigs.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
    motorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.steerMotor.getConfigurator().apply(motorConfigs);

    steerMotor.setControl(new MotionMagicVoltage(rot));
    driveMotor.setVoltage(speedMPS);
  }

  @Override
  public void periodic() {
    
  }
}
