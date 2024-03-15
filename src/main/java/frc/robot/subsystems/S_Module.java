// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class S_Module extends SubsystemBase {
  public SwerveModuleState state;
  public TalonFX driveMotor;
  public TalonFX steerMotor;
  public CANcoder encoder;
  public TalonFXConfiguration steerMotorConfigs;
  public TalonFXConfiguration driveMotorConfigs;

  
  public S_Module(int driveId, int steerID, int canID) {
    this.driveMotor = new TalonFX(driveId); // assign drive motor, steer motor, and cancoder
    this.steerMotor = new TalonFX(steerID);
    this.encoder = new CANcoder(canID);

    this.steerMotorConfigs = new TalonFXConfiguration(); // initialize talon FX configs
    this.driveMotorConfigs = new TalonFXConfiguration();

    var steerSlot0Configs = steerMotorConfigs.Slot0; // tune steer motor
    steerSlot0Configs.kS = 5;
    steerSlot0Configs.kV = 0.12;
    steerSlot0Configs.kA = 0.01;
    steerSlot0Configs.kP = 5;
    steerSlot0Configs.kI = 0;
    steerSlot0Configs.kD = 0.1;

    var driveSlot0Configs = driveMotorConfigs.Slot0; // tune drive motor
    driveSlot0Configs.kS = 5;
    driveSlot0Configs.kV = 0.12;
    driveSlot0Configs.kA = 0.01;
    driveSlot0Configs.kP = 4;
    driveSlot0Configs.kI = 0;
    driveSlot0Configs.kD = 0.1;

    var steerMotionMagicConfigs = steerMotorConfigs.MotionMagic; // tune steer motion magic config
    steerMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
    steerMotionMagicConfigs.MotionMagicAcceleration = 160;
    steerMotionMagicConfigs.MotionMagicJerk = 1600;

    var driveMotionMagicConfigs = driveMotorConfigs.MotionMagic; // tune drive motion magic config
    driveMotionMagicConfigs.MotionMagicCruiseVelocity = 80;
    driveMotionMagicConfigs.MotionMagicAcceleration = 160;
    driveMotionMagicConfigs.MotionMagicJerk = 1600;

    steerMotorConfigs.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID(); // bind steer motor to cancoder and apply configs
    steerMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.steerMotor.getConfigurator().apply(steerMotorConfigs);

    driveMotorConfigs.Feedback.FeedbackRemoteSensorID = this.encoder.getDeviceID(); // bind drive motor to cancoder and apply configs
    driveMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    this.driveMotor.getConfigurator().apply(driveMotorConfigs);
    
//----------------------------------------------------------------------------------------------------//


  }

  public void set(double speed, double targetAngle) {
    double error = Math.abs(getCurrentSteerAngle() - targetAngle);
    this.steerMotor.setControl(new MotionMagicVoltage((error * determineFastestTurn(getCurrentSteerAngle(), targetAngle)) / 360));
    // this.steerMotor.set(error);
      //  this.steerMotor.setControl(new MotionMagicVoltage(targetAngle));
  }
  public int determineFastestTurn(double currentAngle, double targetAngle) {
    int power = 0;
    double cwDifference = (targetAngle - currentAngle + 360) % 360; 
    double ccwDifference = (currentAngle - targetAngle + 360) % 360;

    if(cwDifference < ccwDifference) {
      power = 1;
    } else if (ccwDifference > cwDifference) {
      power = -1;
    }
    return power;
  }
  public double getCurrentSteerAngle() { // 8000 ticks per rot
    double currentPosTicks = this.encoder.getAbsolutePosition().getValueAsDouble();
    double currentPosDegrees = currentPosTicks / SwerveConstants.TICKS_PER_ROT * 360;
    return currentPosDegrees;
    
  }



  public void setDesiredState(SwerveModuleState targetState) {
    SwerveModuleState state = SwerveModuleState.optimize(targetState, new Rotation2d((this.encoder.getAbsolutePosition().getValueAsDouble() * Math.PI/180)));
    double drivePower = SwerveConstants.DRIVE_PID_CONTROLLER.calculate(this.encoder.getVelocity().getValueAsDouble() * SwerveConstants.WHEEL_RADIUS, state.speedMetersPerSecond);
    double driveFeedforward = SwerveConstants.DRIVE_FEED_FORWARD.calculate(state.speedMetersPerSecond);
    double turnPower = SwerveConstants.STEER_PID_CONTROLLER.calculate((this.encoder.getAbsolutePosition().getValueAsDouble() * Math.PI)/180, state.angle.getRadians());
    double turnFeedforward = SwerveConstants.STEER_FEED_FORWARD.calculate(SwerveConstants.STEER_PID_CONTROLLER.getSetpoint());
    // double turnFeedforward = SwerveConstants.STEER_FEED_FORWARD.calculate(state.angle.getRotations());

    this.driveMotor.set(drivePower + driveFeedforward);
    this.steerMotor.set(turnPower + turnFeedforward);


    }


  public void resetSteer() {
      this.steerMotor.setPosition(0);
  }

  @Override
  public void periodic() {
    
  }
}
