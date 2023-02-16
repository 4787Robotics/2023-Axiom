package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.Joystick;

public class CJoystick {
    Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);
    QOL qol2 = new QOL();
    QOL qol3 = new QOL();
    QOL qol4 = new QOL();
    QOL qol5 = new QOL();
    QOL qol6 = new QOL();
    QOL qol7 = new QOL();
    QOL qol9 = new QOL();
    QOL qol11 = new QOL();
    public CJoystick() {

    }
//Joystick POV(the weird knob thing at the top)
    public double getJoystickPOV(){
        if(joystick.getPOV() != -1){
        
            return joystick.getPOV();
        }
        return -1;
    }
// Buttons 1-12
    public boolean joystickButton1Down(){
        if(joystick.getRawButton(1)){
            
            return joystick.getRawButton(1);
        }
        return false;
    }
    public boolean joystickButton2Down(){
        if(joystick.getRawButton(2)){
            
            return joystick.getRawButton(2);
        }
        return false;
    }
    public boolean joystickButton3Down(){
        if(joystick.getRawButton(3)){
            
            return joystick.getRawButton(3);
        }
        return false;
    }
    public boolean joystickButton4Down(){
        if(joystick.getRawButton(4)){
            
            return joystick.getRawButton(4);
        }
        return false;
    }
    public boolean joystickButton5Down(){
        if(joystick.getRawButton(5)){
            
            return joystick.getRawButton(5);
        }
        return false;
    }
    public boolean joystickButton6Down(){
        if(joystick.getRawButton(6)){
            
            return joystick.getRawButton(6);
        }
        return false;
    }
    public boolean joystickButton7Down(){
        if(joystick.getRawButton(7)){
            
            return joystick.getRawButton(7);
        }
        return false;
    }
    public boolean joystickButton8Down(){
        if(joystick.getRawButton(8)){
            
            return joystick.getRawButton(8);
        }
        return false;
    }
    public boolean joystickButton9Down(){
        if(joystick.getRawButton(9)){
            
            return joystick.getRawButton(9);
        }
        return false;
    }
    public boolean joystickButton10Down(){
        if(joystick.getRawButton(10)){
            
            return joystick.getRawButton(10);
        }
        return false;
    }
    public boolean joystickButton11Down(){
        if(joystick.getRawButton(11)){
            
        return joystick.getRawButton(11);
    }
    return false;
    }
    public boolean joystickButton12Down(){
        if(joystick.getRawButton(12)){
           
            return joystick.getRawButton(12);
        }
        return false;
    }
//Joystick axes
    //Joystick X Axis
    public double getJoystickXWithDeadzone(){
        if(joystick.getX() > Constants.joystickDeadzone || joystick.getX() < -Constants.joystickDeadzone){
           
            return joystick.getX();
        }
        return 0;
    }
    //Joystick Y Axis
    public double getJoystickYWithDeadzone(){
        if(joystick.getY() > Constants.joystickDeadzone || joystick.getY() < -Constants.joystickDeadzone){
          
            return joystick.getY();
        }
        return 0;
    }
    //Joystick Rotation
    public double getJoystickRotationWithDeadzone(){
        if(joystick.getTwist() > Constants.joystickRotDeadzone || joystick.getTwist() < -Constants.joystickRotDeadzone){
           
            return joystick.getTwist();
        }
        return 0;
    }
    //Joystick Throttle
    public double getJoystickThrottle(){
            if(joystick.getThrottle() != 1){
               
            return joystick.getThrottle();
            }
        return 1;
    }
}