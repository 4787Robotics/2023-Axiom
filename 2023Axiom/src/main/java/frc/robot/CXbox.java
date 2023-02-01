package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.XboxController;

public class CXbox {
    static XboxController xboxcontroller = new XboxController(0);
    
    CXbox() {

    }
//Dpad buttons
    public static double getXboxDpad(){
        if(xboxcontroller.getPOV() != -1){
            
            return xboxcontroller.getPOV();
        }
        return -1;
    }
//ABXY buttons
    public static boolean XboxADown(){
        if (xboxcontroller.getAButton()) {
            
        }
        return xboxcontroller.getAButton();
    }
    public static boolean XboxBDown(){
        if (xboxcontroller.getBButton()) {
            
        }
        return xboxcontroller.getBButton();
    }
    public static boolean XboxXDown(){
        if (xboxcontroller.getXButton()) {
            
        }
        return xboxcontroller.getXButton();
        
    }
    public static boolean XboxYDown(){
        if (xboxcontroller.getYButton()) {
           
        }
        return xboxcontroller.getYButton();
    }
// R&L bumpers
    public static boolean XboxLBumperDown(){
        if (xboxcontroller.getLeftBumper()) {
           
        }
        return xboxcontroller.getLeftBumper();
    }
    public static boolean XboxRBumperDown(){
        if (xboxcontroller.getRightBumper()) {
            
        }
        return xboxcontroller.getRightBumper();
    }
// R&L stick buttons
    public static boolean XboxLStickDown(){
        if (xboxcontroller.getLeftStickButton()) {
            
        }
        return xboxcontroller.getLeftStickButton();
    }
    public static boolean XboxRStickDown(){
        if (xboxcontroller.getRightStickButton()) {
            
        }
        return xboxcontroller.getRightStickButton();
    }
// left trigger value
    public static double getLeftTriggerWithDeadzone(){
        if (xboxcontroller.getLeftTriggerAxis() > Constants.LEFT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getLeftTriggerAxis();           
        }
        return 0;
    }
// right trigger value
    public static double getRightTriggerWithDeadzone(){
        if (xboxcontroller.getRightTriggerAxis() > Constants.RIGHT_TRIGGER_DEAD_ZONE) {
           
            return xboxcontroller.getRightTriggerAxis();
        }
        return 0;
    }
// left stick X value
    public static double getLeftStickXWithDeadzone(){
        if (xboxcontroller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getLeftX();
        }
        return 0;
    }
// right stick X value
    public static double getRightStickXWithDeadzone() {
        if (xboxcontroller.getRightX() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightX() < -Constants.RIGHT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getRightX();
        }
        return 0;
    }
// left stick Y value
    public static double getLeftStickYWithDeadzone(){
        if (xboxcontroller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
           
            return xboxcontroller.getLeftY();
        }
        return 0;
    }
//right stick Y value
    public static double getRightStickYWithDeadzone(){
        if (xboxcontroller.getRightY() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightY() < Constants.RIGHT_TRIGGER_DEAD_ZONE * -1) {
            
            return xboxcontroller.getRightY();
        }
        return 0;
    }
}
