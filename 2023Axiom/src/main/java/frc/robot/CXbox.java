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
<<<<<<< HEAD
            
=======
            //System.out.println(xboxcontroller.getPOV());
>>>>>>> main
            return xboxcontroller.getPOV();
        }
        return -1;
    }
//ABXY buttons
    public boolean XboxADown(){
        if (xboxcontroller.getAButton()) {
<<<<<<< HEAD
            
=======
            //System.out.println("A");
>>>>>>> main
        }
        return xboxcontroller.getAButton();
    }
    public boolean XboxBDown(){
        if (xboxcontroller.getBButton()) {
<<<<<<< HEAD
            
=======
            //System.out.println("B");
>>>>>>> main
        }
        return xboxcontroller.getBButton();
    }
    public boolean XboxXDown(){
        if (xboxcontroller.getXButton()) {
<<<<<<< HEAD
            
=======
            //System.out.println("X");
>>>>>>> main
        }
        return xboxcontroller.getXButton();
        
    }
    public boolean XboxYDown(){
        if (xboxcontroller.getYButton()) {
<<<<<<< HEAD
           
=======
            //System.out.println("Y");
>>>>>>> main
        }
        return xboxcontroller.getYButton();
    }
// R&L bumpers
    public boolean XboxLBumperDown(){
        if (xboxcontroller.getLeftBumper()) {
<<<<<<< HEAD
           
=======
            //System.out.println("Left Bumper");
>>>>>>> main
        }
        return xboxcontroller.getLeftBumper();
    }
    public boolean XboxRBumperDown(){
        if (xboxcontroller.getRightBumper()) {
<<<<<<< HEAD
            
=======
            //System.out.println("Right Bumper");
>>>>>>> main
        }
        return xboxcontroller.getRightBumper();
    }
// R&L stick buttons
    public boolean XboxLStickDown(){
        if (xboxcontroller.getLeftStickButton()) {
<<<<<<< HEAD
            
=======
            //System.out.println("Left Stick pressed");
>>>>>>> main
        }
        return xboxcontroller.getLeftStickButton();
    }
    public boolean XboxRStickDown(){
        if (xboxcontroller.getRightStickButton()) {
<<<<<<< HEAD
            
=======
            //System.out.println("Right Stick pressed");
>>>>>>> main
        }
        return xboxcontroller.getRightStickButton();
    }
// left trigger value
    public double getLeftTriggerWithDeadzone(){
        if (xboxcontroller.getLeftTriggerAxis() > Constants.LEFT_TRIGGER_DEAD_ZONE) {
<<<<<<< HEAD
            
=======
            //System.out.println("left trigger = "+xboxcontroller.getLeftTriggerAxis());
>>>>>>> main
            return xboxcontroller.getLeftTriggerAxis();           
        }
        return 0;
    }
// right trigger value
    public double getRightTriggerWithDeadzone(){
        if (xboxcontroller.getRightTriggerAxis() > Constants.RIGHT_TRIGGER_DEAD_ZONE) {
<<<<<<< HEAD
           
=======
            //System.out.println("right trigger = "+xboxcontroller.getRightTriggerAxis());
>>>>>>> main
            return xboxcontroller.getRightTriggerAxis();
        }
        return 0;
    }
// left stick X value
    public double getLeftStickXWithDeadzone(){
        if (xboxcontroller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
<<<<<<< HEAD
            
=======
            //System.out.println("left x = "+xboxcontroller.getLeftX());
>>>>>>> main
            return xboxcontroller.getLeftX();
        }
        return 0;
    }
// right stick X value
    public double getRightStickXWithDeadzone() {
        if (xboxcontroller.getRightX() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightX() < -Constants.RIGHT_TRIGGER_DEAD_ZONE) {
<<<<<<< HEAD
            
=======
            //System.out.println("right x = "+xboxcontroller.getRightX());
>>>>>>> main
            return xboxcontroller.getRightX();
        }
        return 0;
    }
// left stick Y value
    public double getLeftStickYWithDeadzone(){
        if (xboxcontroller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || xboxcontroller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
<<<<<<< HEAD
           
=======
            //System.out.println("left y = "+xboxcontroller.getLeftY());
>>>>>>> main
            return xboxcontroller.getLeftY();
        }
        return 0;
    }
//right stick Y value
    public double getRightStickYWithDeadzone(){
        if (xboxcontroller.getRightY() > Constants.RIGHT_TRIGGER_DEAD_ZONE || xboxcontroller.getRightY() < Constants.RIGHT_TRIGGER_DEAD_ZONE * -1) {
<<<<<<< HEAD
            
=======
            //System.out.println("right y = "+xboxcontroller.getRightY());
>>>>>>> main
            return xboxcontroller.getRightY();
        }
        return 0;
    }
}
