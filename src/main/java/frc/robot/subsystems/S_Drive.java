package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveConstants;

public class S_Drive extends SubsystemBase {
 
  public S_Module frontLeftModule, backLeftModule,
                  frontRightModule,  backRightModule;
  
  public S_Drive() {
    frontLeftModule = new S_Module(SwerveConstants.FRONT_LEFT_DRIVE, SwerveConstants.FRONT_LEFT_STEER, SwerveConstants.FRONT_LEFT_CANCODER);
    backLeftModule = new S_Module(SwerveConstants.BACK_LEFT_DRIVE, SwerveConstants.BACK_LEFT_STEER, SwerveConstants.BACK_LEFT_CANCODER);
    frontRightModule = new S_Module(SwerveConstants.FRONT_RIGHT_DRIVE, SwerveConstants.FRONT_RIGHT_STEER, SwerveConstants.FRONT_RIGHT_CANCODER);
    backRightModule = new S_Module(SwerveConstants.BACK_RIGHT_DRIVE, SwerveConstants.BACK_RIGHT_STEER, SwerveConstants.BACK_RIGHT_CANCODER);

  }

   public void drive(XboxController controller) {
        double forwardsVelocity = -MathUtil.applyDeadband(controller.getLeftY(), 0.1); 
        double horizontalVelocity = MathUtil.applyDeadband(controller.getLeftX(), 0.1);
        double rotation = MathUtil.applyDeadband(controller.getRightX(), 0.1); 

        ChassisSpeeds speeds = new ChassisSpeeds(forwardsVelocity, horizontalVelocity, rotation); // collect values and turn into chassisSpeeds
        SwerveModuleState[] moduleStates = SwerveConstants.s_kinematics.toSwerveModuleStates(speeds); // convert chassisSpeeds into swerve module states for our set states method
        setStates(moduleStates[0], moduleStates[1], moduleStates[2], moduleStates[3]); // feed module states into setStates() method
  }  


    public void setStates(SwerveModuleState flState, SwerveModuleState blState, SwerveModuleState frState, SwerveModuleState brState) {

      frontLeftModule.set(flState.speedMetersPerSecond, flState.angle.getRotations());
      backLeftModule.set(blState.speedMetersPerSecond, blState.angle.getRotations());
      frontRightModule.set(frState.speedMetersPerSecond, frState.angle.getRotations());
      backRightModule.set(brState.speedMetersPerSecond, brState.angle.getRotations());


    }

  @Override
  public void periodic() {


    
  }
}
