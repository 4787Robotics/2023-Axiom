package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

public class Balance extends SubsystemBase{
    private double currentRotationPitch = 0; //Current rotation [-180, 180] backward and forward. 1 means leaned 180 degrees forward, -1 180 degrees backward
    private double currentRotationRoll = 0; //Current rotation [-180, 180] tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right
    private double currentHeading = 0;
    private double headingAdjust = 0;
    private boolean headingReady = false;
    private int i = 0;

    private AHRS gyro;

    public Balance() {
        gyro = new AHRS(SPI.Port.kMXP);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Yaw", gyro.getYaw()); //Positive is right, 0 is true north **USUALLY**. There isnt any real rhyme or reason to when it is or isnt, so dont fully trust this
        SmartDashboard.putNumber("Roll", gyro.getRoll()); //Positive is tilted right
        SmartDashboard.putNumber("Pitch", gyro.getPitch()); //Positive is forward
        SmartDashboard.putNumber("Heading Adjust", headingAdjust);

        updateHeading();
        SmartDashboard.putNumber("Heading", currentHeading);
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
    public double getCorrectionSpeed() {
        double correctionSpeed = 0;
        updatePitch();
        updateRoll();
        /*
            TO GO HERE- code for updating correction speed
        */
        return correctionSpeed;
    }

    /**
     * [-180, 180] gets current rotation, backward and forward.
     *
     * @return currentRotationPitch
     */  
    public double getPitch() {
        updatePitch();
        return currentRotationPitch;
    }

    /**
     * [-180, 180] gets current rotation, tilted to left and and right.
     *
     * @return currentRotationRoll
     */  
    public double getRoll() {
        updateRoll();
        return currentRotationRoll;
    }

    /**
     * Gets the AHRS type gyro being used by this object.
     *
     * @return gyro
     */  
    public AHRS getGyro() {
        return gyro;
    }

    private void updatePitch() {
        currentRotationPitch = SmartDashboard.getNumber("Pitch", 0.0);
    }

    private void updateRoll() {
        currentRotationRoll = SmartDashboard.getNumber("Roll", 0.0);
    }

    private void updateHeading() {
        currentHeading = SmartDashboard.getNumber("Yaw", 300) + headingAdjust;
    }

    /**
     * Resets heading so that the new 0 is directly in front of robot
     * DO NOT USE THIS (Unless you ask me and make sure you know what you are doing). 
    */
    public void setHeadingAdjust() {
        this.headingAdjust = 0 - SmartDashboard.getNumber("Yaw", 14.0);
        headingReady = true;
    }

    /**
     * Sets the heading adjust, to change where the zero is, using a double
     * DO NOT USE THIS (Unless you ask me and make sure you know what you are doing)
    */
    public void setHeadingAdjust(double headingAdjust) {
        this.headingAdjust = headingAdjust;
    }

    /**
     * Don't need to use, just some testing stuff for dev
    */
    private void testing() {
        if (headingReady == true) {
            if (currentHeading < -1 || currentHeading > 1) {
                if (currentHeading == (300 + headingAdjust)) {
                    System.out.println("DEFAULTING PROBLEM");
                }
                else {
                    System.out.println("ERROR: " + i);
                    i++;
                    System.out.println(currentHeading + " " + headingAdjust + " " + (currentHeading + headingAdjust));
                }
            }
            else {
                if (Math.random() < Math.random()) {
                    System.out.println("ALL CLEAR");
                    System.out.println(headingAdjust);
                }
            }
        }
    }
}