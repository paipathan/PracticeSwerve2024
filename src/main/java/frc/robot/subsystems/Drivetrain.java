// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private final AHRS gyro;

  public final WPI_TalonSRX shooter1, shooter2, frontLeft, backLeft, frontRight, backRight;
  private final MotorControllerGroup leftMotors, rightMotors;

  public final DifferentialDrive ddrive;
  public DifferentialDriveOdometry odometry;
  public DifferentialDriveKinematics kinematics;

  private Pose2d lastPose;
  private double lastTime;

  public double leftEncoderPosition;
  public double rightEncoderPosition;

  public Drivetrain() {
    gyro = new AHRS(SerialPort.Port.kUSB);

    frontLeft = new WPI_TalonSRX(Constants.MOTOR_L1_ID);
    backLeft = new WPI_TalonSRX(Constants.MOTOR_L2_ID);
    frontRight = new WPI_TalonSRX(Constants.MOTOR_R1_ID);
    backRight = new WPI_TalonSRX(Constants.MOTOR_R2_ID); 


    shooter1 = new WPI_TalonSRX(Constants.SHOOTER_1_ID);
    shooter2 = new WPI_TalonSRX(Constants.SHOOTER_2_ID);
    
    odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getYaw()), leftEncoderPosition / Constants.REVOLUTON_TICKS * Constants.WHEEL_CIRCUM, rightEncoderPosition / Constants.REVOLUTON_TICKS * Constants.WHEEL_CIRCUM);



    leftMotors = new MotorControllerGroup(frontLeft, backLeft);
    leftMotors.setInverted(true);
    rightMotors = new MotorControllerGroup(frontRight, backRight); 

    ddrive = new DifferentialDrive(leftMotors, rightMotors);
    AutoBuilder.configureRamsete(
      this::getPose,
      this::resetPose,
      this::getSpeeds,
      this::setSpeeds,
      new ReplanningConfig(),
      this::flipPath,
      this
    );
  }

  public void move (double power, double offset){
    ddrive.arcadeDrive(power,offset);
  }

  

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void resetPose(Pose2d pose) {
    pose = new Pose2d();
  };

  public boolean flipPath() {
    return DriverStation.getAlliance().equals(Alliance.Red);
  }

  public ChassisSpeeds getSpeeds()
  {
    Pose2d pose = getPose();
    double time = Timer.getFPGATimestamp();

    double deltaX = pose.getX() - lastPose.getX();
    double deltaY = pose.getY() - lastPose.getY();
    double deltaT = time - lastTime;

    lastPose = pose;
    lastTime = time;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(deltaX / deltaT, deltaY / deltaT);
    return kinematics.toChassisSpeeds(wheelSpeeds);
  }

  public void setSpeeds(ChassisSpeeds chassisSpeeds)
  {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    ddrive.tankDrive(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  @Override
  public void periodic() {
    
    leftEncoderPosition = frontLeft.getSelectedSensorPosition()/2;
    rightEncoderPosition = -frontRight.getSelectedSensorPosition();

    odometry.update(new Rotation2d(gyro.getYaw()), leftEncoderPosition / Constants.REVOLUTON_TICKS * Constants.WHEEL_CIRCUM, rightEncoderPosition / Constants.REVOLUTON_TICKS * Constants.WHEEL_CIRCUM);
  
    SmartDashboard.putNumber("Front Left Encoder", leftEncoderPosition);
    SmartDashboard.putNumber("Front Right Encoder", rightEncoderPosition);
    SmartDashboard.updateValues();
 
  }
}