package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveConstants {
    
    public static final double kModuleMaxAngularVelocity = Math.PI;
    public static final double kModuleMaxAngularAcceleration = 2 * Math.PI;

    public static final double SWERVE_WIDTH = 0.61/2; // width of robot is 0.61

    public static final int FRONT_LEFT_DRIVE = 4;
    public static final int FRONT_LEFT_STEER = 5;
    public static final int FRONT_LEFT_CANCODER = 50;
    // public static final double FRONT_LEFT_OFFSET = 291.093750/360;
    public static final double FRONT_LEFT_OFFSET = -0.310302734375;
        public static final Translation2d FL_MOD_LOC = new Translation2d(-SwerveConstants.SWERVE_WIDTH, SwerveConstants.SWERVE_WIDTH);




    public static final int BACK_LEFT_DRIVE = 3;
    public static final int BACK_LEFT_STEER = 1;
    public static final int BACK_LEFT_CANCODER = 53;
    // public static final double BACK_LEFT_OFFSET = 100.54/360;
    public static final double BACK_LEFT_OFFSET = 0.0400390625;
    public static final Translation2d BL_MOD_LOC = new Translation2d(-SwerveConstants.SWERVE_WIDTH, -SwerveConstants.SWERVE_WIDTH);




    public static final int FRONT_RIGHT_DRIVE = 6;
    public static final int FRONT_RIGHT_STEER = 7;
    public static final int FRONT_RIGHT_CANCODER = 52;  
    // public static final double FRONT_RIGHT_OFFSET = 164.41/360;
    public static final double FRONT_RIGHT_OFFSET = -0.27734375;  
    public static final Translation2d FR_MOD_LOC = new Translation2d(SwerveConstants.SWERVE_WIDTH, SwerveConstants.SWERVE_WIDTH);




    public static final int BACK_RIGHT_DRIVE = 8;
    public static final int BACK_RIGHT_STEER = 2; 
    public static final int BACK_RIGHT_CANCODER = 51;
    // public static final double BACK_RIGHT_OFFSET = 325.722656/360;
    public static final double BACK_RIGHT_OFFSET = 0.1015625;
    public static final Translation2d BR_MOD_LOC = new Translation2d(SwerveConstants.SWERVE_WIDTH, -SwerveConstants.SWERVE_WIDTH);




    public static final double kDriveGearRatio = 8.142857142857142;
    public static final double kSteerGearRatio = 21.428571428571427;
    public static final double kWheelRadiusInches = 3.5;
    public static final double kSlipCurrentA = 300.0;
    public static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    public static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    public static final double kSpeedAt12VoltsMps = 6.86;
    public static final double kSteerInertia = 0.00001;
    public static final double kDriveInertia = 0.001;
    public static final double kSteerFrictionVoltage = 0.25;
    public static final double kDriveFrictionVoltage = 0.25;
    public static final double kCoupleRatio = 3.5714285714285716;
    public static final boolean kSteerMotorReversed = false;


    public static final Slot0Configs steerGains = new Slot0Configs()
            .withKP(100).withKI(100).withKD(3)
            .withKS(0).withKV(0).withKA(0);
    
    public static final Slot0Configs driveGains = new Slot0Configs()
            .withKP(100).withKI(50).withKD(3)
            .withKS(0).withKV(0).withKA(0);


    public static final SwerveModuleConstantsFactory s_ConstantFactory = new SwerveModuleConstantsFactory()
      .withDriveMotorGearRatio(SwerveConstants.kDriveGearRatio)
            .withSteerMotorGearRatio(SwerveConstants.kSteerGearRatio)
            .withWheelRadius(SwerveConstants.kWheelRadiusInches)
            .withSlipCurrent(SwerveConstants.kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(SwerveConstants.steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(SwerveConstants.driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(SwerveConstants.kSpeedAt12VoltsMps)
            .withSteerInertia(SwerveConstants.kSteerInertia)
            .withDriveInertia(SwerveConstants.kDriveInertia)
            .withSteerFrictionVoltage(SwerveConstants.kSteerFrictionVoltage)
            .withDriveFrictionVoltage(SwerveConstants.kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
            .withCouplingGearRatio(SwerveConstants.kCoupleRatio)
            .withSteerMotorInverted(SwerveConstants.kSteerMotorReversed);

    public static final SwerveModuleConstants flConst = SwerveConstants.s_ConstantFactory.createModuleConstants(SwerveConstants.FRONT_LEFT_STEER, SwerveConstants.FRONT_LEFT_DRIVE, SwerveConstants.FRONT_LEFT_CANCODER, SwerveConstants.FRONT_LEFT_OFFSET, -SwerveConstants.SWERVE_WIDTH, SwerveConstants.SWERVE_WIDTH, false);   //flmod
    public static final SwerveModuleConstants blConst = SwerveConstants.s_ConstantFactory.createModuleConstants(SwerveConstants.BACK_LEFT_STEER, SwerveConstants.BACK_LEFT_DRIVE, SwerveConstants.BACK_LEFT_CANCODER, SwerveConstants.BACK_LEFT_OFFSET, -SwerveConstants.SWERVE_WIDTH, -SwerveConstants.SWERVE_WIDTH, false);   //blmod
    public static final SwerveModuleConstants frConst = SwerveConstants.s_ConstantFactory.createModuleConstants(SwerveConstants.FRONT_RIGHT_STEER, SwerveConstants.FRONT_RIGHT_DRIVE, SwerveConstants.FRONT_RIGHT_CANCODER, SwerveConstants.FRONT_RIGHT_OFFSET, SwerveConstants.SWERVE_WIDTH, SwerveConstants.SWERVE_WIDTH, false);   //frmod
    public static final SwerveModuleConstants brConst = SwerveConstants.s_ConstantFactory.createModuleConstants(SwerveConstants.BACK_RIGHT_STEER, SwerveConstants.BACK_RIGHT_DRIVE, SwerveConstants.BACK_RIGHT_CANCODER, SwerveConstants.BACK_RIGHT_OFFSET, SwerveConstants.SWERVE_WIDTH, -SwerveConstants.SWERVE_WIDTH, false);  //brmod

    public static final SwerveDriveKinematics s_kinematics = new SwerveDriveKinematics(SwerveConstants.FL_MOD_LOC, SwerveConstants.BL_MOD_LOC, SwerveConstants.FR_MOD_LOC, SwerveConstants.BR_MOD_LOC);

    public static final TalonFXConfiguration flMotorConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration blMotorConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration frMotorConfigs = new TalonFXConfiguration();
    public static final TalonFXConfiguration brMotorConfigs = new TalonFXConfiguration();

    public static final double TICKS_PER_ROT = 4096;
    public static final double WHEEL_RADIUS = 0.0508;  
    public static final double MAX_ANGULAR_VELOCITY = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;

    public static final PIDController DRIVE_PID_CONTROLLER = new PIDController(1, 0, 0);
    public static final  SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(0, 0);

    public static final PIDController STEER_PID_CONTROLLER = new PIDController(0.1, 0, 0);
    public static final SimpleMotorFeedforward STEER_FEED_FORWARD = new SimpleMotorFeedforward(0, 0);

    public static final double MAX_MODULE_SPEED = 3;




}
