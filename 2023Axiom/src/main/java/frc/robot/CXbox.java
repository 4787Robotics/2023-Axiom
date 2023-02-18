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
            //System.out.println(xboxcontroller.getPOV());
            return xboxcontroller.getPOV();
        }
        return -1;
    }
//ABXY buttons
    public boolean XboxADown(){
        if (xboxcontroller.getAButton()) {
            //System.out.println("A");
        }
        return xboxcontroller.getAButton();
    }
    public boolean XboxBDown(){
        if (xboxcontroller.getBButton()) {
            //System.out.println("B");
        }
        return xboxcontroller.getBButton();
    }
    public boolean XboxXDown(){
        if (xboxcontroller.getXButton()) {
            //System.out.println("X");
        }
        return xboxcontroller.getXButton();
        
    }
    public boolean XboxYDown(){
        if (xboxcontroller.getYButton()) {
            //System.out.println("Y");
        }
        return xboxcontroller.getYButton();
    }
// R&L bumpers
    public boolean XboxLBumperDown(){
        if (xboxcontroller.getLeftBumper()) {
            //System.out.println("Left Bumper");
        }
        return xboxcontroller.getLeftBumper();
    }
    public boolean XboxRBumperDown(){
        if (xboxcontroller.getRightBumper()) {
            //System.out.println("Right Bumper");
        }
        return xboxcontroller.getRightBumper();
    }
// R&L stick buttons
    public boolean XboxLStickDown(){
        if (xboxcontroller.getLeftStickButton()) {
            //System.out.println("Left Stick pressed");
        }
        return xboxcontroller.getLeftStickButton();
    }
    public boolean XboxRStickDown(){
        if (xboxcontroller.getRightStickButton()) {
            //System.out.println("Right Stick pressed");
        }
        return xboxcontroller.getRightStickButton();
    }
// left trigger value
    public double getLeftTriggerWithDeadzone(){
        if (xboxcontroller.getLeftTriggerAxis() > Constants.LEFT_TRIGGER_DEAD_ZONE) {
            //System.out.println("left trigger = "+xboxcontroller.getLeftTriggerAxis());
            return xboxcontroller.getLeftTriggerAxis();           
        }
        return 0;
    }
// right trigger value
    public double getRightTriggerWithDeadzone(){
        if (xboxcontroller.getRightTriggerAxis() > Constants.RIGHT_TRIGGER_DEAD_ZONE) {
            //System.out.println("right trigger = "+xboxcontroller.getRightTriggerAxis());
            return xboxcontroller.getRightTriggerAxis();
        }
        return 0;
    }
// left stick X value
    public double getLeftStickXWithDeadzone(){
        if (xboxcontroller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
            //System.out.println("left x = "+xboxcontroller.getLeftX());
            return xboxcontroller.getLeftX();
        }
        return 0;
    }
// right stick X value
    public double getRightStickXWithDeadzone() {
        if (xboxcontroller.getRightX() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightX() < -Constants.RIGHT_TRIGGER_DEAD_ZONE) {
            //System.out.println("right x = "+xboxcontroller.getRightX());
            return xboxcontroller.getRightX();
        }
        return 0;
    }
// left stick Y value
    public double getLeftStickYWithDeadzone(){
        if (xboxcontroller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
            //System.out.println("left y = "+xboxcontroller.getLeftY());
            return xboxcontroller.getLeftY();
        }
        return 0;
    }
//right stick Y value
    public double getRightStickYWithDeadzone(){
        if (xboxcontroller.getRightY() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightY() < Constants.RIGHT_TRIGGER_DEAD_ZONE * -1) {
            //System.out.println("right y = "+xboxcontroller.getRightY());
            return xboxcontroller.getRightY();
        }
        return 0;
    }
}
