//Most of this is copied from TShirt cannon code, so not all of it works atm
//Reminder to add values to constants after talking to electrical
//Currently in the process of switching Motors from Falcons to Spark Maxes :(
// Jan. 23 - I commented out all the errors lol
// Tanmay said we should test the teleop driving stuff
//And the encoders arent that important yet
//FIX IT LATER

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.RelativeEncoder;

//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;

/*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
*/

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase{
    public CANSparkMax m_left1, m_left2, m_right1, m_right2;
    public DifferentialDrive drive;
    //I have no idea if the motor type is brushed or brushless
    //Subject to change
    
  
  
  public DriveTrain(){
    m_left1 = new CANSparkMax(Constants.LEFT_MOTOR_1_ID, MotorType.kBrushless);
    m_left2 = new CANSparkMax(Constants.LEFT_MOTOR_2_ID, MotorType.kBrushless);
    m_right1 = new CANSparkMax(Constants.RIGHT_MOTOR_1_ID, MotorType.kBrushless);
    m_right2 = new CANSparkMax(Constants.RIGHT_MOTOR_2_ID, MotorType.kBrushless);
  
/*
    private static WPI_TalonFX m_left1 = new WPI_TalonFX(Constants.leftMotor1ID);
    private WPI_TalonFX m_left2 = new WPI_TalonFX(Constants.leftMotor2ID);
    private static WPI_TalonFX m_right1 = new WPI_TalonFX(Constants.rightMotor1ID);
    private WPI_TalonFX m_right2 = new WPI_TalonFX(Constants.rightMotor2ID);
    */
    
    /*
    int window_size = 1;
    SensorVelocityMeasPeriod measurement_period = SensorVelocityMeasPeriod.Period_1Ms;

    private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    */
    //makes sure that the wheels on the side are going the same way
    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    drive = new DifferentialDrive(m_left1, m_right1);

    //For the block of errors below:
    //Documentation for Spark Maxes
    //https://codedocs.revrobotics.com/java/index.html
    //Honestly just going to comment it out bc we don't
    //Need it rn, just need to test if
    //The Driving stuff works
    /*
    m_left1.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
    m_left2.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
    m_right1.setFeedbackDevice(FeedbackDevice.IntegratedSensor);
    m_right2.setFeedbackDevice(FeedbackDevice.IntegratedSensor);

    m_left1.setMeasurementPeriod(measurement_period);
    m_left1.configVelocityMeasurementWindow(window_size);
    m_left2.setMeasurementPeriod(measurement_period);
    m_left2.configVelocityMeasurementWindow(window_size);
    m_right1.setMeasurementPeriod(measurement_period);
    m_right1.configVelocityMeasurementWindow(window_size);
    m_right2.setMeasurementPeriod(measurement_period);
    m_right2.configVelocityMeasurementWindow(window_size);

    resetEncoders();
    */
  }
  /*
  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  /*
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftEncoderVelocity = Constants.kDistancePerEncoderCount*(m_left1.getVelocity()*10); 
    double rightEncoderVelocity = Constants.kDistancePerEncoderCount*(m_right1.getVelocity()*10); 

    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
  }
  */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left1.setVoltage(leftVolts);
    m_right1.setVoltage(leftVolts);
    m_left2.setVoltage(leftVolts);
    m_right2.setVoltage(leftVolts);
    drive.feed();
  }
  /*
  public void resetOdometry(Pose2d pose) {
    //double[] initialXandY = {pose.getX(), pose.getY()};
    //SmartDashboard.putNumberArray("Initial Pose", initialXandY);
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }
  
  public static void resetEncoders() {
    m_right1.getPosition(0);
    m_left1.setPosition(0);
  }
  */
    //Turning right/left and moving forward/backward 
    //Add if statements for Fidel's class. Turning + moving forward/backward should be 
    //separate joysticks
    public void driveRobot(double throttle, double turn){
      drive.arcadeDrive(throttle, turn);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      // double leftEncoderPosition = Constants.kDistancePerEncoderCount*m_left1.getPosition();
      // double rightEncoderPosition = Constants.kDistancePerEncoderCount*m_right1.getPosition();
      
      // m_odometry.update(gyro.getRotation2d(), leftEncoderPosition, rightEncoderPosition);
      
    }
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
}
