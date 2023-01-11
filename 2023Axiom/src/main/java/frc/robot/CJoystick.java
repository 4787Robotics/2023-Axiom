package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.Joystick;

public class CJoystick {
    static Joystick joystick = new Joystick(0);
    
    CJoystick() {

    }
    public static boolean joystickButton1Down(){
        if(joystick.getRawButton(1)){
            System.out.println("button 1 = "+joystick.getRawButton(1));
            return joystick.getRawButton(1);
        }
        return false;
    }
    public static boolean joystickButton2Down(){
        if(joystick.getRawButton(2)){
            System.out.println("button 2 = "+joystick.getRawButton(2));
            return joystick.getRawButton(2);
        }
        return false;
    }
    public static boolean joystickButton3Down(){
        if(joystick.getRawButton(3)){
            System.out.println("button 3 = "+joystick.getRawButton(3));
            return joystick.getRawButton(3);
        }
        return false;
    }
    public static boolean joystickButton4Down(){
        if(joystick.getRawButton(4)){
            System.out.println("button 4 = "+joystick.getRawButton(4));
            return joystick.getRawButton(4);
        }
        return false;
    }
    public static boolean joystickButton5Down(){
        if(joystick.getRawButton(5)){
            System.out.println("button 5 = "+joystick.getRawButton(5));
            return joystick.getRawButton(5);
        }
        return false;
    }
    public static boolean joystickButton6Down(){
        if(joystick.getRawButton(6)){
            System.out.println("button 6 = "+joystick.getRawButton(6));
            return joystick.getRawButton(6);
        }
        return false;
    }
    public static boolean joystickButton7Down(){
        if(joystick.getRawButton(7)){
            System.out.println("button 7 = "+joystick.getRawButton(7));
            return joystick.getRawButton(7);
        }
        return false;
    }
    public static boolean joystickButton8Down(){
        if(joystick.getRawButton(8)){
            System.out.println("button 8 = "+joystick.getRawButton(8));
            return joystick.getRawButton(8);
        }
        return false;
    }
    public static boolean joystickButton9Down(){
        if(joystick.getRawButton(9)){
            System.out.println("button 9 = "+joystick.getRawButton(9));
            return joystick.getRawButton(9);
        }
        return false;
    }
    public static boolean joystickButton10Down(){
        if(joystick.getRawButton(10)){
            System.out.println("button 10 = "+joystick.getRawButton(10));
            return joystick.getRawButton(10);
        }
        return false;
    }
    public static boolean joystickButton11Down(){
        if(joystick.getRawButton(11)){
            System.out.println("button 11 = "+joystick.getRawButton(11));
            return joystick.getRawButton(11);
        }
        return false;
    }
    public static boolean joystickButton12Down(){
        if(joystick.getRawButton(12)){
            System.out.println("button 12 = "+joystick.getRawButton(12));
            return joystick.getRawButton(12);
        }
        return false;
    }
    public static double getJoystickXWithDeadzone(){
        if(joystick.getX() > Constants.joystickDeadzone || joystick.getX() < -Constants.joystickDeadzone){
            System.out.println("Joystick X = "+joystick.getX());
            return joystick.getX();
        }
        return 0;
    }
    public static double getJoystickYWithDeadzone(){
        if(joystick.getY() > Constants.joystickDeadzone || joystick.getY() < -Constants.joystickDeadzone){
            System.out.println("Joystick Y = "+joystick.getY());
            return joystick.getY();
        }
        return 0;
    }
    public static double getJoystickRotationWithDeadzone(){
        if(joystick.getTwist() > Constants.joystickRotDeadzone || joystick.getTwist() < -Constants.joystickRotDeadzone){
            System.out.println("Joystick rotation = "+joystick.getTwist());
            return joystick.getTwist();
        }
        return 0;
    }
    public static double getJoystickThrottle(){
            if(joystick.getThrottle()>0){
                System.out.println(joystick.getThrottle());
                return joystick.getThrottle();
            }
        return 0;
    }

}
