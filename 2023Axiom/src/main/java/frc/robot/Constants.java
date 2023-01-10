// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Controller sticks wont respond inside these zones
    public static final float rightStickDeadzone = 0.9f;
    public static final float leftStickDeadzone = 0.9f;
    public static final float rightTriggerDeadzone = 0.7f;
    public static final float leftTriggerDeadzone = 0.7f;
    //Motor IDs
    public static final int rightMotor2ID = 0;
    public static final int rightMotor1ID = 0;
    public static final int leftMotor2ID = 0;
    public static final int leftMotor1ID = 0;
    //chloe's stuff
    public static final boolean left_side_inverted = false;
    public static final boolean right_side_inverted = false;
    //may need to change the Constant for these three later
    public static int motor_arm = 0; 
    public static int motor_leftGrip = 1;
    public static int motor_rightGrip = 2;
}
