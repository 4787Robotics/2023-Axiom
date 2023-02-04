package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.Joystick;

public class CJoystick {
    static Joystick joystick = new Joystick(0);
    
    CJoystick() {

    }
//Joystick POV(the weird knob thing at the top)
    public static double getJoystickPOV(){
    if(joystick.getPOV() != -1){
       
        return joystick.getPOV();
    }
    return -1;
    }
// Buttons 1-12
    public static boolean joystickButton1Down(){
        QOL.toggle(joystick.getRawButtonPressed(1), joystick.getRawButton(1));
        if(joystick.getRawButton(1)){
            return joystick.getRawButton(1);
        }
        return false;
    }
    public static boolean joystickButton2Down(){
        if(joystick.getRawButton(2)){
            
            return joystick.getRawButton(2);
        }
        return false;
    }
    public static boolean joystickButton3Down(){
        if(joystick.getRawButton(3)){
            
            return joystick.getRawButton(3);
        }
        return false;
    }
    public static boolean joystickButton4Down(){
        if(joystick.getRawButton(4)){
            
            return joystick.getRawButton(4);
        }
        return false;
    }
    public static boolean joystickButton5Down(){
        if(joystick.getRawButton(5)){
            
            return joystick.getRawButton(5);
        }
        return false;
    }
    public static boolean joystickButton6Down(){
        if(joystick.getRawButton(6)){
           
            return joystick.getRawButton(6);
        }
        return false;
    }
    public static boolean joystickButton7Down(){
        if(joystick.getRawButton(7)){
            
            return joystick.getRawButton(7);
        }
        return false;
    }
    public static boolean joystickButton8Down(){
        if(joystick.getRawButton(8)){
            
            return joystick.getRawButton(8);
        }
        return false;
    }
    public static boolean joystickButton9Down(){
        if(joystick.getRawButton(9)){
           
            return joystick.getRawButton(9);
        }
        return false;
    }
    public static boolean joystickButton10Down(){
        if(joystick.getRawButton(10)){
            
            return joystick.getRawButton(10);
        }
        return false;
    }
    public static boolean joystickButton11Down(){
        if(joystick.getRawButton(11)){
            
            return joystick.getRawButton(11);
        }
        return false;
    }
    public static boolean joystickButton12Down(){
        if(joystick.getRawButton(12)){
           
            return joystick.getRawButton(12);
        }
        return false;
    }
//Joystick axes
    //Joystick X Axis
    public static double getJoystickXWithDeadzone(){
        if(joystick.getX() > Constants.joystickDeadzone || joystick.getX() < -Constants.joystickDeadzone){
           
            return joystick.getX();
        }
        return 0;
    }
    //Joystick Y Axis
    public static double getJoystickYWithDeadzone(){
        if(joystick.getY() > Constants.joystickDeadzone || joystick.getY() < -Constants.joystickDeadzone){
          
            return joystick.getY();
        }
        return 0;
    }
    //Joystick Rotation
    public static double getJoystickRotationWithDeadzone(){
        if(joystick.getTwist() > Constants.joystickRotDeadzone || joystick.getTwist() < -Constants.joystickRotDeadzone){
           
            return joystick.getTwist();
        }
        return 0;
    }
    //Joystick Throttle
    public static double getJoystickThrottle(){
            if(joystick.getThrottle() != 1){
               
            return joystick.getThrottle();
            }
        return 1;
    }
}