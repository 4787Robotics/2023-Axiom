package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Robot;

public class Balance extends SubsystemBase{
    private double currentRotationPitch = 0; //Current rotation [-180, 180] backward and forward. 1 means leaned 180 degrees forward, -1 180 degrees backward
    private double currentRotationRoll = 0; //Current rotation [-180, 180] tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right
    private double currentRotationYaw = 0;
    private double currentHeading = 0;
    private double currentLinearHeading = 0;
    
    private double headingAdjust = 0;
    private double linearHeadingAdjust = 36000;
    private boolean headingReady = false;

    private int i = 0; //For testing, unnecessary otherwise

    private static AHRS gyro;
    private PIDController PID;

    public Balance() {
        gyro = new AHRS(SPI.Port.kMXP);
        PID = new PIDController(0.03, 0.03, 0.01425);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Yaw", getYaw()); //Positive is right, 0 is true north **USUALLY**. There isnt any real rhyme or reason to when it is or isnt, so dont fully trust this
        SmartDashboard.putNumber("Roll", getPitch()); //Positive is tilted right
        SmartDashboard.putNumber("Pitch", -getRoll()); //Positive is forward
        SmartDashboard.putNumber("Heading Adjust", headingAdjust);
        SmartDashboard.putNumber("LinearHeading", getLinearHeading());

        updateHeading();
        SmartDashboard.putNumber("Heading", currentHeading);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
     * [-180, 180] gets current rotation, left and right, sometimes in relation to true north. 
     * Unstable- do not use. Use getHeading instead
     *
     * @return currentRotationYaw
     */  
    public double getYaw() {
        updateYaw();
        return currentRotationYaw;
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
     * [-180, 180] gets current rotation, tilted to left and and right.
     *
     * @return currentRotationRoll
     */  
    public double getHeading() {
        updateHeading();
        return currentHeading;
    }

    public double getLinearHeading() {
        updateLinearHeading();
        return currentLinearHeading;
    }
 
    public double getTestHeading() {
        return Math.IEEEremainder(getYaw(), 360);
    }

    /**
     * Gets the AHRS type gyro being used by this object.
     *
     * @return gyro
     */  
    public static AHRS getGyro() {
        return gyro;
    }

    /**
     * Gets the PID controller being used by this object.
     *
     * @return PID
     */ 
    public PIDController getPID() {
        return PID;
    }

    public double calculatePID(double currentMeasurement, double goalPoint) {
        return PID.calculate(currentMeasurement, goalPoint);
    }

    private void updateYaw() {
        currentRotationYaw = gyro.getYaw();
    }

    private void updatePitch() {
        currentRotationPitch = -gyro.getRoll(); //Negative and getRoll instead of getPitch because NavX is placed onto the robot 90 degrees off
    }

    private void updateRoll() {
        currentRotationRoll = gyro.getPitch(); //Negative and getRoll instead of getPitch because NavX is placed onto the robot 90 degrees off
    }

    private void updateHeading() {
        currentHeading = getYaw() + headingAdjust;
    }
    
    private void updateLinearHeading() {
        currentLinearHeading = gyro.getAngle() + linearHeadingAdjust;
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