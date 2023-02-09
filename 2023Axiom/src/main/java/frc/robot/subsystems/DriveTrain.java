//Most of this is copied from TShirt cannon code, so not all of it works atm
//Reminder to add values to constants after talking to electrical
//Currently in the process of switching Motors from Falcons to Spark Maxes :(
// Jan. 23 - I commented out all the errors lol
// Tanmay said we should test the teleop driving stuff
//And the encoders arent that important yet
//FIX IT LATER

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.subsystems.Balance;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import java.lang.Math.*;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;

/*
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
*/

// import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

public class DriveTrain extends SubsystemBase{
    public static WPI_TalonFX m_left1;
    public WPI_TalonFX m_left2;
    public static WPI_TalonFX m_right1;
    public WPI_TalonFX m_right2;
    public DifferentialDrive drive;
    //I have no idea if the motor type is brushed or brushless
    //Subject to change
    private static final Timer timer = new Timer();
  
    private double previous_time;
  
    private double totalLeftWheelDistanceMeters;
    private double totalRightWheelDistanceMeters;

    public DifferentialDriveOdometry m_odometry;
    public AHRS gyro;
  
  public DriveTrain(){
    timer.start();

    m_left1 = new WPI_TalonFX(Constants.LEFT_MOTOR_1_ID); //Front left
    m_left2 = new WPI_TalonFX(Constants.LEFT_MOTOR_2_ID); //Back left
    m_right1 = new WPI_TalonFX(Constants.RIGHT_MOTOR_1_ID); //Front left
    m_right2 = new WPI_TalonFX(Constants.RIGHT_MOTOR_2_ID); //Back left
  
    m_left1.enableVoltageCompensation(true);
    m_left2.enableVoltageCompensation(true);
    m_right1.enableVoltageCompensation(true);
    m_right2.enableVoltageCompensation(true);

    m_left1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 0.5));
    m_left2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 0.5));
    m_right1.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 0.5));
    m_right2.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 35, 40, 0.5));
    
    m_left1.configOpenloopRamp(0.2); // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
    m_left2.configOpenloopRamp(0.2); // (helps keep robot from rocking around violently every time driver stops)
    m_right1.configOpenloopRamp(0.2);
    m_right2.configOpenloopRamp(0.2);

   
    
    SensorVelocityMeasPeriod measurement_period = SensorVelocityMeasPeriod.Period_1Ms;
    Integer window_size = 100;

    AHRS gyro = Balance.getGyro();
    m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), totalLeftWheelDistanceMeters, totalRightWheelDistanceMeters);
    //makes sure that the wheels on the side are going the same way
    m_left1.setInverted(TalonFXInvertType.CounterClockwise);
    m_left2.setInverted(TalonFXInvertType.CounterClockwise);
    m_right1.setInverted(TalonFXInvertType.Clockwise);
    m_right2.setInverted(TalonFXInvertType.Clockwise);

    m_left2.setNeutralMode(NeutralMode.Brake);
    m_left1.setNeutralMode(NeutralMode.Coast);
    m_right2.setNeutralMode(NeutralMode.Brake);
    m_right1.setNeutralMode(NeutralMode.Coast);

    m_left2.follow(m_left1);
    m_right2.follow(m_right1);

    drive = new DifferentialDrive(m_left1, m_right1);

    //For the block of errors below:
    //Documentation for Spark Maxes
    //https://codedocs.revrobotics.com/java/index.html
    //Honestly just going to comment it out bc we don't
    //Need it rn, just need to test if
    //The Driving stuff works
    
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
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
      gyro.getVelocityX(), 
      gyro.getVelocityY(), 
      gyro.getVelocityZ()
    );
    return Constants.K_DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds);
  }
  
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left1.setVoltage(leftVolts);
    m_right1.setVoltage(leftVolts);
    m_left2.setVoltage(leftVolts);
    m_right2.setVoltage(leftVolts);
    drive.feed();
  }
  
  private void setWheelPositionZero() {
    totalLeftWheelDistanceMeters = 0;
    totalRightWheelDistanceMeters = 0;
  } 

  
  public void resetOdometry(Pose2d pose) {
    setWheelPositionZero();
    m_odometry.resetPosition(gyro.getRotation2d(), totalLeftWheelDistanceMeters, totalRightWheelDistanceMeters, pose);
  }
  
  public static void resetEncoders() {
    m_right1.getPosition(0);
    m_left1.setPosition(0);
  }
  */
  //Turning right/left and moving forward/backward 
  //Add if statements for Fidel's class. Turning + moving forward/backward should be 
  //separate joysticks
  public void driveRobot(boolean squareInputs, double throttle, double turn){
    if (squareInputs) {
      drive.arcadeDrive(throttle * Math.abs(throttle), turn * Math.abs(turn)); //squaring inputs to make robot not go as hard forward at lower levels, to avoid it stroking out
    }
    else {
      drive.arcadeDrive(throttle, turn);
    }
  }
    m_left1.configClearPositionOnLimitF(true, 0);
    m_left1.configClearPositionOnLimitR(true, 0);
    m_left1.configClearPositionOnQuadIdx(true, 0);

    m_right1.configClearPositionOnLimitF(true, 0);
    m_right1.configClearPositionOnLimitR(true, 0);
    m_right1.configClearPositionOnQuadIdx(true, 0);
  }
  
    //Turning right/left and moving forward/backward 
    //Add if statements for Fidel's class. Turning + moving forward/backward should be 
    //separate joysticks
    public void driveRobot(double throttle, double turn){
      drive.arcadeDrive(throttle, turn);
    }
    private void trackLeftAndRightDistance(DifferentialDriveWheelSpeeds wheelSpeeds) {
      double leftVelocity = wheelSpeeds.leftMetersPerSecond;
  
      double rightVelocity = wheelSpeeds.rightMetersPerSecond;
  
      double current_time = Timer.getFPGATimestamp();
  
      double timeElapsedBetweenLoops = current_time - previous_time;
  
      double leftWheelDistanceMeters = leftVelocity*timeElapsedBetweenLoops;
      double rightWheelDistanceMeters = rightVelocity*timeElapsedBetweenLoops;
  
      totalLeftWheelDistanceMeters += leftWheelDistanceMeters;
      totalRightWheelDistanceMeters += rightWheelDistanceMeters;
    }
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      // double leftEncoderPosition = Constants.kDistancePerEncoderCount*m_left1.getPosition();
      // double rightEncoderPosition = Constants.kDistancePerEncoderCount*m_right1.getPosition();
      
      // m_odometry.update(gyro.getRotation2d(), leftEncoderPosition, rightEncoderPosition);
      SmartDashboard.putNumber("Left Output",m_left1.get());
      SmartDashboard.putNumber("Right Output",m_right1.get());
      SmartDashboard.putNumber("Left Position",totalLeftWheelDistanceMeters);
      SmartDashboard.putNumber("Right Position",totalRightWheelDistanceMeters);
    }
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
    public void autonomousDrive(double speed, double turnSpeed) {
      drive.arcadeDrive(speed,turnSpeed,false);
    }
    /**
     * Sets the speeds for each side of tank drive individually (for easier usage with encoders).
     * @param leftSpeed [-1.0..1.0]
     * @param rightSpeed [-1.0..1.0]
     */
  }