//Most of the Constants need to be changed
//Do so when you acquire the numbers 

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //ports
    public static int JOYSTICK_PORT = 0;
    public static int XCONTROLLER_PORT= 1;

    //Limelight
    public static double LIMELIGHT_APRILTAG_DOUBLE_STATION_HEIGHT = 36; //cm
    public static double LIMELIGHT_APRILTAG_GRID_HEIGHT = 59; //cm
    public static double LIMELIGHT_REFLECTIVETAPE_LOW_HEIGHT = 61; //cm
    public static double LIMELIGHT_REFLECTIVETAPE_HIGH_HEIGHT = 111; //cm
    public static double LIMELIGHT_MOUNT_ANGLE = 0; //degrees
    public static double LIMELIGHT_LENS_HEIGHT = 33.02; //cm
    public static int ALL_APRILTAG_IDS_PIPELINE = 6;

    //Motors
    public static int MOTOR_ARM_1 = 5;
    public static int MOTOR_ARM_2 = 6; //Copies arm 1
    public static int MOTOR_LEFT_GRIP = 8;
    public static int MOTOR_RIGHT_GRIP = 7; //does the opposite of left Grip
    public static int MOTOR_MOVE_GRIP = 10; //Snow Blower
    public static int MOTOR_ARM_HOLD = 9; //Starting motor
    
    //Position of where the arm should be for each grid level. (in rotations)
    public static double LOW_LEVEL = 0; 
    public static double MID_LEVEL = 58; 
    public static double HIGH_LEVEL = 82.5; //was 11.5361827601

    //One motor rotation is 1.1680107% of one full arm rotation. (36.288 degrees per rotation)
    
    //public static boolean LEFT_SIDE_INVERTED = true;
    //public static boolean RIGHT_SIDE_INVERTED = !LEFT_SIDE_INVERTED;

    //The constants below will need to be changed later - Talk to electrical
    public static int LEFT_MOTOR_1_ID = 2; //Front Left
    public static int LEFT_MOTOR_2_ID = 1; //Back Left

    public static int RIGHT_MOTOR_1_ID = 3; //Front right
    public static int RIGHT_MOTOR_2_ID = 4; //Back right

    //Constants for CXbox and CJoystick
    //Controller sticks wont respond inside these zones
    public static final float RIGHT_STICK_DEAD_ZONE = 0.02f;
    public static final float LEFT_STICK_DEAD_ZONE = 0.05f;
    public static final float RIGHT_TRIGGER_DEAD_ZONE = 0.1f;
    public static final float LEFT_TRIGGER_DEAD_ZONE = 0.1f;
    //Joystick
    public static final float joystickRotDeadzone = 0.1f;
    public static final float joystickDeadzone = 0.2f;

    //Constants for Autonomous
    public static final double KS_VOLTS = 0.21878;
    public static final double KV_VOLT_SECONDS_PER_METER = 1.0966;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.27491;

    public static final double KP_DRIVE_VEL = 1.5588;

    //Differential Drive Kinematics
    public static final double K_TRACK_WIDTH_METERS = 0.5760466; //22.679 inch
    public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(K_TRACK_WIDTH_METERS);

    // Max Trajectory Velocity/Acceleration
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 4;
    public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2; // limits max acceleration during autonomous

    // Ramsete Parameters
    public static final double K_RAMSETE_B = 2;
    public static final double K_RAMSETE_A = 0;
    public static final double K_RAMSETE_ZETA = 0.7;
    
    public static final double K_WHEEL_DIAMETER_METERS = 0.1524; //6 inches
    public static final int K_ENCODER_CPR = 0;

    public static final double K_GEAR_RATIO = 0;
    public static final double K_ENCODER_COUNTS_PER_FULL_WHEEL_TURN = K_GEAR_RATIO*K_ENCODER_CPR;
    public static final double K_DISTANCE_PER_ENCODER_COUNT = (K_WHEEL_DIAMETER_METERS * Math.PI) / K_ENCODER_COUNTS_PER_FULL_WHEEL_TURN;

    public final int path1 = 1;
    public final int path2 = 2;
    public final int path3 = 3;
    public final int path4 = 4;
    public final int path5 = 5;
    public final int path6 = 6;
    public final int path7 = 7;
    public final int path8 = 8;
    public final int path9 = 9;
    public final int path10 = 10;
    public final int path11 = 11;
    public final int path12 = 12;
    public final int path13 = 13;
    /*
    1 = chargeStation
    2 = getCone2a
    3 = getCone2b
    4 = getCone2a
    5 = getCone2b
    6 = moveBackv1
    7 = placeCone2a
    8 = placeCone2b
    9 = placeCone2a
    10 = placeCone2b
    11 = preloadedCone2a
    12 = preloadedCone2b
    13 = preloadedCubev1
    */
}
