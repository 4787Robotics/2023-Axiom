package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.Joystick;

public class CJoystick {
    static Joystick joystick = new Joystick(0);
    
    CJoystick() {

    }
    public static double getJoystickXWithDeadzone(){
        if(joystick.getX() > Constants.joystickDeadzone || joystick.getX() < -Constants.joystickDeadzone){
            System.out.println("JoystickX = "+joystick.getX());
            return joystick.getX();
        }
        return 0;
    }
    public static double getJoystickYWithDeadzone(){
        if(joystick.getY() > Constants.joystickDeadzone || joystick.getY() < -Constants.joystickDeadzone){
            System.out.println("JoystickY = "+joystick.getY());
            return joystick.getY();
        }
        return 0;
    }
    public static double getJoystickRotationWithDeadzone(){
        if(joystick.getTwist() > Constants.joystickRotDeadzone || joystick.getTwist() < -Constants.joystickRotDeadzone){
            System.out.println("JoystickX = "+joystick.getTwist());
            return joystick.getTwist();
        }
        return 0;
    }
}
