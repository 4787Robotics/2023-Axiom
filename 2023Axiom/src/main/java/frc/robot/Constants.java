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

    public static int motor_arm = 0; //may need to change the Constant for these three later
    public static int motor_leftGrip = 1;
    public static int motor_rightGrip = 2;

    
    public static boolean left_side_inverted = true;
    public static boolean right_side_inverted = !left_side_inverted;

    //The constants below will need to be changed later - Talk to electrical
    public static int leftMotor1ID = 4;
    public static int leftMotor2ID = 3;

    public static int rightMotor1ID = 1;
    public static int rightMotor2ID = 2;

    //Constants for CXbox and CJoystick
    //Controller sticks wont respond inside these zones
    public static final float rightStickDeadzone = 0.9f;
    public static final float leftStickDeadzone = 0.9f;
    public static final float rightTriggerDeadzone = 0.7f;
    public static final float leftTriggerDeadzone = 0.7f;
    //Joystick
    public static final float joystickRotDeadzone = 0.9f;
    public static final float joystickDeadzone = 0.9f;

    //Constants for Autonomous
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;

    public static final double kPDriveVel = 0;

    //Differential Drive Kinematics
    public static final double kTrackwidthMeters = 0;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    // Max Trajectory Velocity/Acceleration
    public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;

    // Ramsete Parameters
    public static final double kRamseteB = 0;
    public static final double kRamseteZeta = 0;

    public static final double kWheelDiameterMeters = 0;
    public static final int kEncoderCPR = 0;

    public static final double kGearRatio = 0;

    public static final double kEncoderCountsPerFullWheelTurn = kGearRatio*kEncoderCPR;
    public static final double kDistancePerEncoderCount = (kWheelDiameterMeters * Math.PI) / kEncoderCountsPerFullWheelTurn;
}
