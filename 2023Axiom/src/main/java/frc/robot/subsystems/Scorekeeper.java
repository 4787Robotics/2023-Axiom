package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CJoystick;
import frc.robot.CXbox;

public class Scorekeeper {
    public static void updateDashboard() {
    //Joystick
        //movable stuff
            SmartDashboard.putNumber("Joystick POV val", CJoystick.getJoystickPOV() );
            SmartDashboard.putNumber("Joystick Rot val", CJoystick.getJoystickRotationWithDeadzone() );
            SmartDashboard.putNumber("Joystick Thrtl val", CJoystick.getJoystickThrottle() );
            SmartDashboard.putNumber("Joystick X val", CJoystick.getJoystickXWithDeadzone() );
            SmartDashboard.putNumber("Joystick Y wal", CJoystick.getJoystickYWithDeadzone() );

        //buttons
            SmartDashboard.putBoolean("B1", CJoystick.joystickButton1Down() );
            SmartDashboard.putBoolean("B2", CJoystick.joystickButton2Down() );
            SmartDashboard.putBoolean("B3", CJoystick.joystickButton3Down() );
            SmartDashboard.putBoolean("B4", CJoystick.joystickButton4Down() );
            SmartDashboard.putBoolean("B5", CJoystick.joystickButton5Down() );
            SmartDashboard.putBoolean("B6", CJoystick.joystickButton6Down() );
            SmartDashboard.putBoolean("B7", CJoystick.joystickButton7Down() );
            SmartDashboard.putBoolean("B8", CJoystick.joystickButton8Down() );
            SmartDashboard.putBoolean("B9", CJoystick.joystickButton9Down() );
            SmartDashboard.putBoolean("B10", CJoystick.joystickButton10Down() );
            SmartDashboard.putBoolean("B11", CJoystick.joystickButton11Down() );
            SmartDashboard.putBoolean("B12", CJoystick.joystickButton12Down() );

    //Xbox Controller
        //Movable Stuff
            SmartDashboard.putNumber("LSX", CXbox.getLeftStickXWithDeadzone() );
            SmartDashboard.putNumber("LSY", CXbox.getLeftStickYWithDeadzone() );
            SmartDashboard.putNumber("RSX", CXbox.getRightStickXWithDeadzone() );
            SmartDashboard.putNumber("RSY", CXbox.getRightStickYWithDeadzone() );
            SmartDashboard.putNumber("LT", CXbox.getRightStickXWithDeadzone() );
            SmartDashboard.putNumber("RT", CXbox.getRightStickYWithDeadzone() );
            SmartDashboard.putNumber("DPad", CXbox.getXboxDpad() );


        //Buttons
            SmartDashboard.putBoolean("A", CXbox.XboxADown());
            SmartDashboard.putBoolean("B", CXbox.XboxBDown() );
            SmartDashboard.putBoolean("X", CXbox.XboxXDown() );
            SmartDashboard.putBoolean("Y", CXbox.XboxYDown() );
            SmartDashboard.putBoolean("LJB", CXbox.XboxLStickDown() );
            SmartDashboard.putBoolean("RJB", CXbox.XboxRStickDown());
            SmartDashboard.putBoolean("LB", CXbox.XboxLBumperDown() );
            SmartDashboard.putBoolean("RB", CXbox.XboxRBumperDown() );
            

    }
}
