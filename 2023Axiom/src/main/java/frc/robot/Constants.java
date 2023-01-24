//Most of the Constants need to be changed
//Do so when you acquire the numbers 

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static int MOTOR_ARM = 0; //may need to change the Constant for these three later
    public static int MOTOR_LEFT_GRIP = 1;
    public static int MOTOR_RIGHT_GRIP = 2;

    
    public static boolean LEFT_SIDE_INVERTED = true;
    public static boolean RIGHT_SIDE_INVERTED = !left_side_inverted;

    //The constants below will need to be changed later - Talk to electrical
    public static int LEFT_MOTOR_1_ID = 4;
    public static int LEFT_MOTOR_2_ID = 3;

    public static int RIGHT_MOTOR_1_ID = 1;
    public static int RIGHT_MOTOR_2_ID = 2;

    //Constants for CXbox and CJoystick
    //Controller sticks wont respond inside these zones
    public static final float RIGHT_STICK_DEAD_ZONE = 0.9f;
    public static final float LEFT_STICK_DEAD_ZONE = 0.9f;
    public static final float RIGHT_TRIGGER_DEAD_ZONEe = 0.7f;
    public static final float LEFT_TRIGGER_DEAD_ZONE = 0.7f;
    //Joystick
    public static final float joystickRotDeadzone = 0.9f;
    public static final float joystickDeadzone = 0.9f;

    //Constants for Autonomous
    public static final double KS_VOLTS = 0;
    public static final double KV_VOLT_SECONDS_PER_METER = 0;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0;

    public static final double KP_DRIVE_VEL = 0;

    //Differential Drive Kinematics
    public static final double K_TRACK_WIDTH_METERS = 0;
    public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double K_MAX_SPEED_METERS_PER_SECOND = 0;
    public static final double K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0;

    // Ramsete Parameters
    public static final double K_RAMSETE_B = 0;
    public static final double K_RAMSETE_A = 0;

    
    K_WHEEL_DIAMETER_METERS
    public static final double K_WHEEL_DIAMETER_METERS = 0;
    public static final int K_ENCODER_CPR = 0;

    public static final double K_GEAR_RATION = 0;
    public static final double K_ENCODER_COUNTS_PER_FULL_WHEEL_TURN = kGearRatio*kEncoderCPR;
    public static final double K_DISTANCE_PER_ENCODER_COUNT = (kWheelDiameterMeters * Math.PI) / kEncoderCountsPerFullWheelTurn;
}
