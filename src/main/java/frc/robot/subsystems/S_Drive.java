package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveConstants;

public class S_Drive extends SubsystemBase {
 
  public S_Module frontLeftModule, backLeftModule,
                  frontRightModule,  backRightModule;

  public XboxController controller;
  
  public S_Drive() {
    this.controller = new XboxController(Constants.XBOX_DRIVE_CONTROLLER_PORT);
    frontLeftModule = new S_Module(SwerveConstants.FRONT_LEFT_DRIVE, SwerveConstants.FRONT_LEFT_STEER, SwerveConstants.FRONT_LEFT_CANCODER);
    backLeftModule = new S_Module(SwerveConstants.BACK_LEFT_DRIVE, SwerveConstants.BACK_LEFT_STEER, SwerveConstants.BACK_LEFT_CANCODER);
    frontRightModule = new S_Module(SwerveConstants.FRONT_RIGHT_DRIVE, SwerveConstants.FRONT_RIGHT_STEER, SwerveConstants.FRONT_RIGHT_CANCODER);
    backRightModule = new S_Module(SwerveConstants.BACK_RIGHT_DRIVE, SwerveConstants.BACK_RIGHT_STEER, SwerveConstants.BACK_RIGHT_CANCODER);

  }

   public void drive() {

        ChassisSpeeds speeds = new ChassisSpeeds(-controller.getLeftY(), controller.getLeftX(), controller.getRightX()); // collect values and turn into chassisSpeeds
        SwerveModuleState[] moduleStates = SwerveConstants.s_kinematics.toSwerveModuleStates(speeds); // convert chassisSpeeds into swerve module states for our set states method
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, SwerveConstants.MAX_MODULE_SPEED); // makes sure the modules cant go past a set max speed

        frontLeftModule.setDesiredState(moduleStates[0]);
        backLeftModule.setDesiredState(moduleStates[1]);
        frontRightModule.setDesiredState(moduleStates[2]);
        backRightModule.setDesiredState(moduleStates[3]);

  }  



    public void resetPostion() {
      if(controller.getYButton()) {
      frontLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
      backLeftModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
      frontRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
      backRightModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(0)));
      }
    }

    

  @Override
  public void periodic() {

    SmartDashboard.putNumber("fl angle", frontLeftModule.steerMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("fl dist", frontRightModule.driveMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("bl angle", backLeftModule.steerMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("bl dist", backLeftModule.driveMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("fr angle", frontRightModule.steerMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("fr dist", frontRightModule.driveMotor.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("br angle", backRightModule.steerMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("br dist", backRightModule.driveMotor.getPosition().getValueAsDouble());

    
  }
}
