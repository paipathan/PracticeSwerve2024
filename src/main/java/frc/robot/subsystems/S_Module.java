// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class S_Module extends SubsystemBase {
  public TalonFX driveMotor;
  public TalonFX steerMotor;
  public CANcoder encoder;
  public TalonFXConfiguration steerMotorConfigs;
  public TalonFXConfiguration driveMotorConfigs;
  
  public S_Module(int driveId, int steerID, int canID) {
    this.driveMotor = new TalonFX(driveId);
    this.steerMotor = new TalonFX(steerID);
    this.encoder = new CANcoder(canID);
    this.steerMotorConfigs = new TalonFXConfiguration();

    var steerSlot0Configs = steerMotorConfigs.Slot0;
    steerSlot0Configs.kS = 0;
    steerSlot0Configs.kV = 0.12;
    steerSlot0Configs.kA = 0.01;
    steerSlot0Configs.kP = 5;
    steerSlot0Configs.kI = 0;
    steerSlot0Configs.kD = 0.1;

    var steerMotionMagicConfigs = steerMotorConfigs.MotionMagic;
    steerMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
    steerMotionMagicConfigs.MotionMagicAcceleration = 160;
    steerMotionMagicConfigs.MotionMagicJerk = 1600;

    steerMotorConfigs.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
    steerMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.steerMotor.getConfigurator().apply(steerMotorConfigs);
//----------------------------------------------------------------------------------------------------//
    this.driveMotorConfigs = new TalonFXConfiguration();

    var driveSlot0Configs = driveMotorConfigs.Slot0;
    driveSlot0Configs.kS = 0;
    driveSlot0Configs.kV = 0.12;
    driveSlot0Configs.kA = 0.01;
    driveSlot0Configs.kP = 4;
    driveSlot0Configs.kI = 0;
    driveSlot0Configs.kD = 0.1;

    var driveMotionMagicConfigs = driveMotorConfigs.MotionMagic;
    driveMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
    driveMotionMagicConfigs.MotionMagicAcceleration = 160;
    driveMotionMagicConfigs.MotionMagicJerk = 1600;


    driveMotorConfigs.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID();
    driveMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.driveMotor.getConfigurator().apply(driveMotorConfigs);

   

  }

  public void set(SwerveModuleState state) {

    // state = SwerveModuleState.optimize(state, state.angle);

    this.driveMotor.set(state.speedMetersPerSecond);
    this.steerMotor.setPosition(state.angle.getRotations());

    
  }


  public void resetSteer() {
      this.steerMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    
  }
}
