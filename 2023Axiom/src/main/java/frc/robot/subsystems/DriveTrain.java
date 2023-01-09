//Most of this is copied from TShirt cannon code, so not all of it works atm
package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;

//Download library later
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

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
    

  public driveTrain(){
    
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
    //Turning right/left and moving forward/backward 
    //Some of this may have to be changed when Fidel's user input class is implemented
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

  
}
