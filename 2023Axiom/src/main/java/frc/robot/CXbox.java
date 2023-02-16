package frc.robot;

import java.util.logging.ConsoleHandler;

import edu.wpi.first.wpilibj.XboxController;

public class CXbox {
    static XboxController xboxcontroller = new XboxController(Constants.XCONTROLLER_PORT);
    
    public CXbox() {

    }
//Dpad buttons
    public double getXboxDpad(){
        if(xboxcontroller.getPOV() != -1){
            
            return xboxcontroller.getPOV();
        }
        return -1;
    }
//ABXY buttons
    public boolean XboxADown(){
        if (xboxcontroller.getAButton()) {
            
        }
        return xboxcontroller.getAButton();
    }
    public boolean XboxBDown(){
        if (xboxcontroller.getBButton()) {
            
        }
        return xboxcontroller.getBButton();
    }
    public boolean XboxXDown(){
        if (xboxcontroller.getXButton()) {
            
        }
        return xboxcontroller.getXButton();
        
    }
    public boolean XboxYDown(){
        if (xboxcontroller.getYButton()) {
           
        }
        return xboxcontroller.getYButton();
    }
// R&L bumpers
    public boolean XboxLBumperDown(){
        if (xboxcontroller.getLeftBumper()) {
           
        }
        return xboxcontroller.getLeftBumper();
    }
    public boolean XboxRBumperDown(){
        if (xboxcontroller.getRightBumper()) {
            
        }
        return xboxcontroller.getRightBumper();
    }
// R&L stick buttons
    public boolean XboxLStickDown(){
        if (xboxcontroller.getLeftStickButton()) {
            
        }
        return xboxcontroller.getLeftStickButton();
    }
    public boolean XboxRStickDown(){
        if (xboxcontroller.getRightStickButton()) {
            
        }
        return xboxcontroller.getRightStickButton();
    }
// left trigger value
    public double getLeftTriggerWithDeadzone(){
        if (xboxcontroller.getLeftTriggerAxis() > Constants.LEFT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getLeftTriggerAxis();           
        }
        return 0;
    }
// right trigger value
    public double getRightTriggerWithDeadzone(){
        if (xboxcontroller.getRightTriggerAxis() > Constants.RIGHT_TRIGGER_DEAD_ZONE) {
           
            return xboxcontroller.getRightTriggerAxis();
        }
        return 0;
    }
// left stick X value
    public double getLeftStickXWithDeadzone(){
        if (xboxcontroller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getLeftX();
        }
        return 0;
    }
// right stick X value
    public double getRightStickXWithDeadzone() {
        if (xboxcontroller.getRightX() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightX() < -Constants.RIGHT_TRIGGER_DEAD_ZONE) {
            
            return xboxcontroller.getRightX();
        }
        return 0;
    }
// left stick Y value
    public double getLeftStickYWithDeadzone(){
        if (xboxcontroller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
           
            return xboxcontroller.getLeftY();
        }
        return 0;
    }
//right stick Y value
    public double getRightStickYWithDeadzone(){
        if (xboxcontroller.getRightY() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightY() < Constants.RIGHT_TRIGGER_DEAD_ZONE * -1) {
            
            return xboxcontroller.getRightY();
        }
        return 0;
    }
}
