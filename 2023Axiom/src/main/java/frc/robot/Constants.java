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

    public static final int MOTOR_ARM_1 = 5;
    public static final int MOTOR_ARM_2 = 6;
    public static final int MOTOR_LEFT_HAND = 1; //Chnage needed for two
    public static final int MOTOR_RIGHT_HAND = 2;

    public static final float RIGHT_STICK_DEAD_ZONE = 0.9f;
    public static final float LEFT_STICK_DEAD_ZONE = 0.9f;
    public static final float RIGHT_TRIGGER_DEAD_ZONE = 0.7f;
    public static final float LEFT_TRIGGER_DEAD_ZONE = 0.7f;
}
