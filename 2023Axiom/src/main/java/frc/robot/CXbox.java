package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.XboxController;

public class CXbox {
    static XboxController xboxcontroller = new XboxController(0);
    
    CXbox() {

    }
//Dpad buttons
    public static double XboxDpad(){
        if(xboxcontroller.getPOV() != -1){
            System.out.println(xboxcontroller.getPOV());
            return xboxcontroller.getPOV();
        }
        return 0;
    }
//ABXY buttons
    public static boolean XboxADown(){
        if (xboxcontroller.getAButton()) {
            System.out.println("A");
        }
        return xboxcontroller.getAButton();
    }
    public static boolean XboxBDown(){
        if (xboxcontroller.getBButton()) {
            System.out.println("B");
        }
        return xboxcontroller.getBButton();
    }
    public static boolean XboxXDown(){
        if (xboxcontroller.getXButton()) {
            System.out.println("X");
        }
        return xboxcontroller.getXButton();
        
    }
    public static boolean XboxYDown(){
        if (xboxcontroller.getYButton()) {
            System.out.println("Y");
        }
        return xboxcontroller.getYButton();
    }
// R&L bumpers
    public static boolean XboxLBumperDown(){
        if (xboxcontroller.getLeftBumper()) {
            System.out.println("Left Bumper");
        }
        return xboxcontroller.getLeftBumper();
    }
    public static boolean XboxRBumperDown(){
        if (xboxcontroller.getRightBumper()) {
            System.out.println("Right Bumper");
        }
        return xboxcontroller.getRightBumper();
    }
// R&L stick buttons
    public static boolean XboxLStickDown(){
        if (xboxcontroller.getLeftStickButton()) {
            System.out.println("Left Stick pressed");
        }
        return xboxcontroller.getLeftStickButton();
    }
    public static boolean XboxRStickDown(){
        if (xboxcontroller.getRightStickButton()) {
            System.out.println("Right Stick pressed");
        }
        return xboxcontroller.getRightStickButton();
    }
// left trigger value
    public static double getLeftTriggerWithDeadzone(){
        if (xboxcontroller.getLeftTriggerAxis() > Constants.leftTriggerDeadzone) {
            System.out.println("left trigger = "+xboxcontroller.getLeftTriggerAxis());
            return xboxcontroller.getLeftTriggerAxis();           
        }
        return 0;
    }
// right trigger value
    public static double getRightTriggerWithDeadzone(){
        if (xboxcontroller.getRightTriggerAxis() > Constants.rightTriggerDeadzone) {
            System.out.println("right trigger = "+xboxcontroller.getRightTriggerAxis());
            return xboxcontroller.getRightTriggerAxis();
        }
        return 0;
    }
// left stick X value
    public static double getLeftStickXWithDeadzone(){
        if (xboxcontroller.getLeftX() > Constants.leftStickDeadzone || xboxcontroller.getLeftX() < -Constants.leftStickDeadzone) {
            System.out.println("left x = "+xboxcontroller.getLeftX());
            return xboxcontroller.getLeftX();
        }
        return 0;
    }
// right stick X value
    public static double getRightStickXWithDeadzone() {
        if (xboxcontroller.getRightX() > Constants.rightStickDeadzone || xboxcontroller.getRightX() < -Constants.rightStickDeadzone) {
            System.out.println("right x = "+xboxcontroller.getRightX());
            return xboxcontroller.getRightX();
        }
        return 0;

    }
// left stick Y value
    public static double getLeftStickYWithDeadzone(){
        if (xboxcontroller.getLeftY() > Constants.leftStickDeadzone || xboxcontroller.getLeftY() < -Constants.leftStickDeadzone) {
            System.out.println("left y = "+xboxcontroller.getLeftY());
            return xboxcontroller.getLeftY();
        }
        return 0;
    }
//right stick Y value
    public static double getRightStickYWithDeadzone(){
        if (xboxcontroller.getRightY() > Constants.rightStickDeadzone || xboxcontroller.getRightY() < Constants.rightStickDeadzone * -1) {
            System.out.println("right y = "+xboxcontroller.getRightY());
            return xboxcontroller.getRightY();
        }
        return 0;
    }
}