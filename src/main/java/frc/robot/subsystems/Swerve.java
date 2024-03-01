package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveConstants;


public class Swerve extends SubsystemBase {

public SwerveModule flModule, blModule,
                    frModule, brModule;

public SwerveDriveOdometry odometry;
public AHRS gyro;


  public Swerve() {
    gyro = new AHRS(SerialPort.Port.kUSB);

    flModule = new SwerveModule(SwerveConstants.flConst, ""); 
    blModule = new SwerveModule(SwerveConstants.blConst, "");   
    frModule = new SwerveModule(SwerveConstants.frConst, "");
    brModule = new SwerveModule(SwerveConstants.brConst, "");

    this.odometry = new SwerveDriveOdometry(SwerveConstants.s_kinematics, gyro.getRotation2d(), getSwerveModulePositions(), new Pose2d());
  }


  public void driveWithJoystick(XboxController controller) {
        
        double ySpeed = MathUtil.applyDeadband(-controller.getLeftY(), 0.1); 
        double xSpeed = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
        double rotation = MathUtil.applyDeadband(controller.getRightX(), 0.1); 

        ChassisSpeeds speeds = new ChassisSpeeds(ySpeed, xSpeed, rotation);
        SwerveModuleState[] moduleStates = SwerveConstants.s_kinematics.toSwerveModuleStates(speeds);
        setSwerveModuleStates(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]);
        
    }               
    
  

  public void setSwerveModuleStates(SwerveModuleState frontLeft, SwerveModuleState backLeft, 
                                    SwerveModuleState frontRight, SwerveModuleState backRight) 
  {

        // SwerveConstants.flMotorConfigs.Feedback.FeedbackRemoteSensorID = flModule.getCANcoder().getDeviceID();
        // SwerveConstants.flMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // flModule.getSteerMotor().getConfigurator().apply(SwerveConstants.flMotorConfigs);

        // SwerveConstants.blMotorConfigs.Feedback.FeedbackRemoteSensorID = blModule.getCANcoder().getDeviceID();
        // SwerveConstants.blMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // blModule.getSteerMotor().getConfigurator().apply(SwerveConstants.blMotorConfigs);

        // SwerveConstants.frMotorConfigs.Feedback.FeedbackRemoteSensorID = frModule.getCANcoder().getDeviceID();
        // SwerveConstants.frMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // frModule.getSteerMotor().getConfigurator().apply(SwerveConstants.frMotorConfigs);

        // SwerveConstants.brMotorConfigs.Feedback.FeedbackRemoteSensorID = brModule.getCANcoder().getDeviceID();
        // SwerveConstants.brMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // brModule.getSteerMotor().getConfigurator().apply(SwerveConstants.brMotorConfigs);

        // flModule.getSteerMotor().setControl(new MotionMagicVoltage(frontLeft.angle.getRotations()));
        // flModule.getDriveMotor().setVoltage(frontLeft.speedMetersPerSecond);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        // blModule.getSteerMotor().setControl(new MotionMagicVoltage(backLeft.angle.getRotations()));
        // blModule.getDriveMotor().setVoltage(backLeft.speedMetersPerSecond);                                                                                                                                                     

        // frModule.getSteerMotor().setControl(new MotionMagicVoltage(frontRight.angle.getRotations()));
        // frModule.getDriveMotor().setVoltage(frontRight.speedMetersPerSecond);                              

        // brModule.getSteerMotor().setControl(new MotionMagicVoltage(backRight.angle.getRotations()));
        // brModule.getDriveMotor().setVoltage(backRight.speedMetersPerSecond);


        // flModule.getSteerMotor().setVoltage(frontLeft.angle.getRadians());
        // flModule.getDriveMotor().setVoltage(frontLeft.speedMetersPerSecond);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
        // blModule.getSteerMotor().setVoltage(backLeft.angle.getRadians());
        // blModule.getDriveMotor().setVoltage(backLeft.speedMetersPerSecond);                                                                                                                                                     

        // frModule.getSteerMotor().setVoltage(frontRight.angle.getRadians());
        // frModule.getDriveMotor().setVoltage(frontRight.speedMetersPerSecond);                              

        // brModule.getSteerMotor().setVoltage(backRight.angle.getRadians());
        // brModule.getDriveMotor().setVoltage(backRight.speedMetersPerSecond);

  }


                   
  public void updateOdometry() 
  {
        odometry = new SwerveDriveOdometry(SwerveConstants.s_kinematics, gyro.getRotation2d(), getSwerveModulePositions());
        SmartDashboard.putNumber("Ppose X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Rotation", odometry.getPoseMeters().getRotation().getDegrees());
  }

  public SwerveModulePosition[] getSwerveModulePositions() 
  {
    return new SwerveModulePosition[] {
            flModule.getPosition(true),
            blModule.getPosition(true),
            frModule.getPosition(true),
            brModule.getPosition(true)
    };
}

  public void logCAN() {

    SmartDashboard.putNumber("fl angle", flModule.getSteerMotor().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("fl dist", flModule.getDriveMotor().getPosition().getValueAsDouble());

    SmartDashboard.putNumber("bl angle", blModule.getSteerMotor().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("bl dist", blModule.getDriveMotor().getPosition().getValueAsDouble());

    SmartDashboard.putNumber("fr angle", frModule.getSteerMotor().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("fr dist", frModule.getDriveMotor().getPosition().getValueAsDouble());

    SmartDashboard.putNumber("br angle", brModule.getSteerMotor().getPosition().getValueAsDouble());
    SmartDashboard.putNumber("br dist", brModule.getDriveMotor().getPosition().getValueAsDouble());

  }
  

  @Override
  public void periodic() {
    updateOdometry();
    logCAN();
  }

}
