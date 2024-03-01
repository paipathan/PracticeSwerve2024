// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

//

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // drive motors: 0, 16, 2, 3 for inf recharge game bot
    // drive motors: 1, 2, 3, 4 for 2022 bot
    // public static final int MOTOR_L1_ID = 1;

    //Motors/Controller

    

    public static final int MOTOR_L1_ID = 14;
    public static final int MOTOR_L2_ID = 15;
    public static final int MOTOR_R1_ID = 4;
    public static final int MOTOR_R2_ID = 10; /// KITBOT CONSTANTS

    public static final int SHOOTER_1_ID = 3;
    public static final int SHOOTER_2_ID = 5;

    public static final int ELEVATOR_MOTOR_1 = 2; 
    public static final int ELEVATOR_MOTOR_2 = 7;

    // public static final int MOTOR_L1_ID = 3; // front left
    // public static final int MOTOR_L2_ID = 5; // back left
    // public static final int MOTOR_R1_ID = 2; // front right
    // public static final int MOTOR_R2_ID = 7; // back right ELEVATOR CONSTANTS



    public static final int MAX_VELOCITY = 3;






    public static int revID = 6;
    public static int intakeID = 5;
    public static int slider = 4;

    public static final int JOYSTICK_PORT = 0;
    public static final int XBOX_DRIVE_CONTROLLER_PORT = 0;
    public static final int GYRO_PIGEON = 12;
    public static final double WHEEL_CIRCUM = 0.31;
    public static final int REVOLUTON_TICKS = 5046;

    public static final double cpr = 4096; //Counts Per Revolution
    public static final double whd = 6; //Wheel Radius needs to be measured
    public static final double TRACK_WIDTH = 0.57;
    private static final double WHEEL_BASE = 0.5; 
    public static final int BANNER_MOTOR_TEST = 9;
    public static final int BANNER_SENSOR_PORT = 1;

    public static final boolean invertGyro = false;

    public static final I2C.Port i2cPort = I2C.Port.kOnboard;

    public static final int INTAKE_MOTOR_ID = 44; // on the real robot



}