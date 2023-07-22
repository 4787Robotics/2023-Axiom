package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CJoystick;
import frc.robot.CXbox;

public class Scorekeeper {
    static CJoystick Joystick = new CJoystick();
    static CXbox Xbox = new CXbox();
    static ScoringArea score = new ScoringArea();

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
        SmartDashboard.putBoolean("B2", Joystick.joystickButton2Down() );
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
    //*
    //Movable Stuff
        SmartDashboard.putNumber("LSX", Xbox.getLeftStickXWithDeadzone() );
        SmartDashboard.putNumber("LSY", Xbox.getLeftStickYWithDeadzone() );
        SmartDashboard.putNumber("RSX", Xbox.getRightStickXWithDeadzone() );
        SmartDashboard.putNumber("RSY", Xbox.getRightStickYWithDeadzone() );
        SmartDashboard.putNumber("LT", Xbox.getLeftTriggerWithDeadzone() );
        SmartDashboard.putNumber("RT", Xbox.getRightTriggerWithDeadzone() );
        SmartDashboard.putNumber("DPad", Xbox.getXboxDpad() );


    //Buttons
        SmartDashboard.putBoolean("A", Xbox.XboxADown());
        SmartDashboard.putBoolean("B", Xbox.XboxBDown() );
        SmartDashboard.putBoolean("X", Xbox.XboxXDown() );
        SmartDashboard.putBoolean("Y", Xbox.XboxYDown() );
        SmartDashboard.putBoolean("LJB", Xbox.XboxLStickDown() );
        SmartDashboard.putBoolean("RJB", Xbox.XboxRStickDown());
        SmartDashboard.putBoolean("LB", Xbox.XboxLBumperDown() );
        SmartDashboard.putBoolean("RB", Xbox.XboxRBumperDown() );
    //*/
    //score
        SmartDashboard.putString("Chosen Position", score.goalKeeperPos());
        
        SmartDashboard.putBoolean("0,0,0", score.goal(0,0,0));
        SmartDashboard.putBoolean("0,0,1", score.goal(0,0,1));
        SmartDashboard.putBoolean("0,0,2", score.goal(0,0,2));
        SmartDashboard.putBoolean("0,1,0", score.goal(0,1,0));
        SmartDashboard.putBoolean("0,1,1", score.goal(0,1,1));
        SmartDashboard.putBoolean("0,1,2", score.goal(0,1,2));
        SmartDashboard.putBoolean("0,2,0", score.goal(0,2,0));
        SmartDashboard.putBoolean("0,2,1", score.goal(0,2,1));
        SmartDashboard.putBoolean("0,2,2", score.goal(0,2,2));
        
        SmartDashboard.putBoolean("1,0,0", score.goal(1,0,0));
        SmartDashboard.putBoolean("1,0,1", score.goal(1,0,1));
        SmartDashboard.putBoolean("1,0,2", score.goal(1,0,2));
        SmartDashboard.putBoolean("1,1,0", score.goal(1,1,0));
        SmartDashboard.putBoolean("1,1,1", score.goal(1,1,1));
        SmartDashboard.putBoolean("1,1,2", score.goal(1,1,2));
        SmartDashboard.putBoolean("1,2,0", score.goal(1,2,0));
        SmartDashboard.putBoolean("1,2,1", score.goal(1,2,1));
        SmartDashboard.putBoolean("1,2,2", score.goal(1,2,2));
        
        SmartDashboard.putBoolean("2,0,0", score.goal(2,0,0));
        SmartDashboard.putBoolean("2,0,1", score.goal(2,0,1));
        SmartDashboard.putBoolean("2,0,2", score.goal(2,0,2));
        SmartDashboard.putBoolean("2,1,0", score.goal(2,1,0));
        SmartDashboard.putBoolean("2,1,1", score.goal(2,1,1));
        SmartDashboard.putBoolean("2,1,2", score.goal(2,1,2));
        SmartDashboard.putBoolean("2,2,0", score.goal(2,2,0));
        SmartDashboard.putBoolean("2,2,1", score.goal(2,2,1));
        SmartDashboard.putBoolean("2,2,2", score.goal(2,2,2));
    }   
}



