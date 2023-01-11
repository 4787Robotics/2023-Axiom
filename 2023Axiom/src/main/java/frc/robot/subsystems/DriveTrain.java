//Most of this is copied from TShirt cannon code, so not all of it works atm
//Reminder to add values to constants after talking to electrical
 
package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;


import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrain extends SubsystemBase{
    DifferentialDrive drive;
    //This stuff will probably change a bit as we actually figure out the motors ids
    private static WPI_TalonFX m_left1 = new WPI_TalonFX(Constants.leftMotor1ID);
    private WPI_TalonFX m_left2 = new WPI_TalonFX(Constants.leftMotor2ID);
    private static WPI_TalonFX m_right1 = new WPI_TalonFX(Constants.rightMotor1ID);
    private WPI_TalonFX m_right2 = new WPI_TalonFX(Constants.rightMotor2ID);
  
    boolean left_side_inverted = Constants.left_side_inverted;
    boolean right_side_inverted = Constants.right_side_inverted;
  
    int window_size = 1;
    SensorVelocityMeasPeriod measurement_period = SensorVelocityMeasPeriod.Period_1Ms;

    private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
  
  public DriveTrain(){

    //makes sure that the wheels on the side are going the same way
    m_left2.follow(m_left1);
    m_right1.follow(m_right2);

    drive = new DifferentialDrive(m_left1, m_right1);

    m_left1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_left2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_right1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    m_right2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    m_left1.configVelocityMeasurementPeriod(measurement_period);
    m_left1.configVelocityMeasurementWindow(window_size);
    m_left2.configVelocityMeasurementPeriod(measurement_period);
    m_left2.configVelocityMeasurementWindow(window_size);
    m_right1.configVelocityMeasurementPeriod(measurement_period);
    m_right1.configVelocityMeasurementWindow(window_size);
    m_right2.configVelocityMeasurementPeriod(measurement_period);
    m_right2.configVelocityMeasurementWindow(window_size);

    resetEncoders();
  }

  public double getHeading(){
    return gyro.getRotation2d().getDegrees();
  }
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftEncoderVelocity = Constants.kDistancePerEncoderCount*(m_left1.getSelectedSensorVelocity()*10); 
    double rightEncoderVelocity = Constants.kDistancePerEncoderCount*(m_right1.getSelectedSensorVelocity()*10); 

    return new DifferentialDriveWheelSpeeds(leftEncoderVelocity, rightEncoderVelocity);
  }
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left1.setVoltage(leftVolts);
    m_right1.setVoltage(rightVolts);
    drive.feed();
  }
  public void resetOdometry(Pose2d pose) {
    //double[] initialXandY = {pose.getX(), pose.getY()};
    //SmartDashboard.putNumberArray("Initial Pose", initialXandY);
    resetEncoders();
    m_odometry.resetPosition(pose, gyro.getRotation2d());
  }
  public static void resetEncoders() {
    m_right1.setSelectedSensorPosition(0);
    m_left1.setSelectedSensorPosition(0);
  }
    //Turning right/left and moving forward/backward 
    //Add if statements for Fidel's class. Turning + moving forward/backward should be 
    //separate joysticks
    public void turnRight(double axis){
      System.out.println("Left Joystick Y axis = " + axis);
      drive.arcadeDrive(axis, 0);
    }
    public void turnLeft(double axis){
      System.out.println("Left Joystick Y axis =" + axis);
      drive.arcadeDrive(axis, 0);
    }
    public void moveBackward(double axis){
      System.out.println("Left Joystick X axis = " + axis);
      drive.arcadeDrive(0, axis);
    }
    public void moveForward(double axis){
      System.out.println("Left Joystick X axis = " + axis);
      drive.arcadeDrive(0, axis);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      double leftEncoderPosition = Constants.kDistancePerEncoderCount*m_left1.getSelectedSensorPosition();
      double rightEncoderPosition = Constants.kDistancePerEncoderCount*m_right1.getSelectedSensorPosition();
      
      m_odometry.update(gyro.getRotation2d(), leftEncoderPosition, rightEncoderPosition);
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
}
