package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CJoystick;
import frc.robot.CXbox;

public class Scorekeeper {
    public static void updateDashboard() {
        //movable stuff
        SmartDashboard.putNumber("Joystick POV value", CJoystick.getJoystickPOV() );
        SmartDashboard.putNumber("Joystick Rotation value", CJoystick.getJoystickRotationWithDeadzone() );
        SmartDashboard.putNumber("Joystick Throttle value", CJoystick.getJoystickThrottle() );
        SmartDashboard.putNumber("Joystick X value", CJoystick.getJoystickXWithDeadzone() );
        SmartDashboard.putNumber("Joystick Y value", CJoystick.getJoystickYWithDeadzone() );

        //buttons
        SmartDashboard.putBoolean("Joystick B1 value", CJoystick.joystickButton1Down() );
        SmartDashboard.putBoolean("Joystick B2 value", CJoystick.joystickButton2Down() );
        SmartDashboard.putBoolean("Joystick B3 value", CJoystick.joystickButton3Down() );
        SmartDashboard.putBoolean("Joystick B4 value", CJoystick.joystickButton4Down() );
        SmartDashboard.putBoolean("Joystick B5 value", CJoystick.joystickButton5Down() );
        SmartDashboard.putBoolean("Joystick B6 value", CJoystick.joystickButton6Down() );
        SmartDashboard.putBoolean("Joystick B7 value", CJoystick.joystickButton7Down() );
        SmartDashboard.putBoolean("Joystick B8 value", CJoystick.joystickButton8Down() );
        SmartDashboard.putBoolean("Joystick B9 value", CJoystick.joystickButton9Down() );
        SmartDashboard.putBoolean("Joystick B10 value", CJoystick.joystickButton10Down() );
        SmartDashboard.putBoolean("Joystick B11 value", CJoystick.joystickButton11Down() );
        SmartDashboard.putBoolean("Joystick B12 value", CJoystick.joystickButton12Down() );
    }
}
