package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Balance extends SubsystemBase{
    private static int currentRotationPitch = 0; //Current rotation [-1, 1] backward and forward. 1 means leaned 180 degrees forward, -1 180 degrees backward
    private static int currentRotationRoll = 0; //Current rotation [-1, 1] tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * [-1, 1] a number that should be plugged into arcadeDrive to get the right speed to move the robot to keep it balanced
     *
     * @return correctionSpeed
     */  
    public static int getCorrectionSpeed() {
        int correctionSpeed = 0;
        updatePitch();
        updateRoll();
        /*
            TO GO HERE- code for updating correction speed
        */
        return correctionSpeed;
    }

    
    /**
     * [-1, 1] gets current rotation, backward and forward. 1 means leaned 180 degrees forward, -1 180 degrees backward
     *
     * @return currentRotationPitch
     */  
    public static int getPitch() {
        updatePitch();
        return currentRotationPitch;
    }

    /**
     * [-1, 1] gets current rotation, tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right
     *
     * @return currentRotationRoll
     */  
    public static int getRoll() {
        updateRoll();
        return currentRotationRoll;
    }

    private static void updatePitch() {
        
    }

    private static void updateRoll() {

    }
}