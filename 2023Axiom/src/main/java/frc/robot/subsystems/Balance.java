package frc.robot.subsystems;

<<<<<<< HEAD
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Balance extends SubsystemBase{
    private static int currentRotationPitch = 0; //Current rotation [-1, 1] backward and forward. 1 means leaned 180 degrees forward, -1 180 degrees backward
    private static int currentRotationRoll = 0; //Current rotation [-1, 1] tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right
=======
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
    private double currentRotationYaw = 0;
    private double currentHeading = 0;
    private double headingAdjust = 0;
    private boolean headingReady = false;
    private int i = 0;

    private AHRS gyro;

    public Balance() {
        gyro = new AHRS(SPI.Port.kMXP);
    }
>>>>>>> alex-new-branch

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
<<<<<<< HEAD
=======
        SmartDashboard.putNumber("Yaw", getYaw()); //Positive is right, 0 is true north **USUALLY**. There isnt any real rhyme or reason to when it is or isnt, so dont fully trust this
        SmartDashboard.putNumber("Roll", getRoll()); //Positive is tilted right
        SmartDashboard.putNumber("Pitch", getPitch()); //Positive is forward
        SmartDashboard.putNumber("Heading Adjust", headingAdjust);

        updateHeading();
        SmartDashboard.putNumber("Heading", currentHeading);
>>>>>>> alex-new-branch
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    /**
<<<<<<< HEAD
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
=======
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
>>>>>>> alex-new-branch
        updatePitch();
        return currentRotationPitch;
    }

    /**
<<<<<<< HEAD
     * [-1, 1] gets current rotation, tilted to left and and right. 1 means leaned 180 degrees left, -1 180 degrees right
     *
     * @return currentRotationRoll
     */  
    public static int getRoll() {
=======
     * [-180, 180] gets current rotation, tilted to left and and right.
     *
     * @return currentRotationRoll
     */  
    public double getRoll() {
>>>>>>> alex-new-branch
        updateRoll();
        return currentRotationRoll;
    }

<<<<<<< HEAD
    private static void updatePitch() {
        
    }

    private static void updateRoll() {

    }
}
=======
    /**
     * [-180, 180] gets current rotation, tilted to left and and right.
     *
     * @return currentRotationRoll
     */  
    public double getHeading() {
        updateHeading();
        return currentHeading;
    }

    /**
     * Gets the AHRS type gyro being used by this object.
     *
     * @return gyro
     */  
    public AHRS getGyro() {
        return gyro;
    }

    private void updateYaw() {
        currentRotationYaw = gyro.getYaw();
    }

    private void updatePitch() {
        currentRotationPitch = gyro.getPitch();
    }

    private void updateRoll() {
        currentRotationRoll = gyro.getRoll();
    }

    private void updateHeading() {
        currentHeading = getYaw() + headingAdjust;
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
>>>>>>> alex-new-branch
