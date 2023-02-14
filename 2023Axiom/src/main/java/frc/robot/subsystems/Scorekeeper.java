package frc.robot.subsystems;

//import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CJoystick;
//import frc.robot.CXbox;
import frc.robot.QOL;

public class Scorekeeper {
    static CJoystick Joystick = new CJoystick();
    public static void updateDashboard() {
    
    //Joystick
    //*
    //movable stuff
        SmartDashboard.putNumber("Joystick POV val", Joystick.getJoystickPOV() );
        SmartDashboard.putNumber("Joystick Rot val", Joystick.getJoystickRotationWithDeadzone() );
        SmartDashboard.putNumber("Joystick Thrtl val", Joystick.getJoystickThrottle() );
        SmartDashboard.putNumber("Joystick X val", Joystick.getJoystickXWithDeadzone() );
        SmartDashboard.putNumber("Joystick Y wal", Joystick.getJoystickYWithDeadzone() );

    //buttons
        SmartDashboard.putBoolean("B1", Joystick.joystickButton1Down() );
        SmartDashboard.putNumber("B2", Joystick.joystickButton2Down() );
        SmartDashboard.putBoolean("B3", Joystick.joystickButton3Down() );
        SmartDashboard.putBoolean("B4", Joystick.joystickButton4Down() );
        SmartDashboard.putBoolean("B5", Joystick.joystickButton5Down() );
        SmartDashboard.putBoolean("B6", Joystick.joystickButton6Down() );
        SmartDashboard.putBoolean("B7", Joystick.joystickButton7Down() );
        SmartDashboard.putBoolean("B8", Joystick.joystickButton8Down() );
        SmartDashboard.putBoolean("B9", Joystick.joystickButton9Down() );
        SmartDashboard.putBoolean("B10", Joystick.joystickButton10Down() );
        SmartDashboard.putBoolean("B11", Joystick.joystickButton11Down() );
        SmartDashboard.putBoolean("B12", Joystick.joystickButton12Down() );
    //*/
//Xbox Controller\
    /*
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
    //*/
    }   
}



