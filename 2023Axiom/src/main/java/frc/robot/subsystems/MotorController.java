package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;


public class MotorController extends SubsystemBase {
  private CANSparkMax leftHand;
  private CANSparkMax RightHand;
  private MotorController ArmMotor;
  private RelativeEncoder armEncoder;
  private PIDController PID;
    if (voltages.length != 4) {
      System.out.println("Need 4 arguments");
      return;
    }
      // Big Arm Motor
  private CANSparkMax m_arm = new CANSparkMax(Constants.MOTOR_ARM, MotorType.kBrushless);
  armEncoder = m_arm.getEncoder(); //for PID
  PID = new PIDController(1, 1, 1); CHANGE BEFORE TEST

  // Controls wheels that suck em up
  private WPI_TalonFX m_left_wheel = new WPI_TalonFX(Constants.MOTOR_LEFT_WHEEL);
  private WPI_TalonFX m_right_wheel = new WPI_TalonFX(Constants.MOTOR_RIGHT_WHEEL);

  // Makes left motor go the opposite of the right motor
  m_left_wheel.setInverted(TalonFXInvertType.Clockwise); 
   // ewqeKeeps the motors in place and stops them frowm moving without input
  m_arm.setIdleMode(IdleMode.kBrake);
  m_left_wheel.setNeutralMode(NeutralMode.Brake);
  m_right_wheel.setNeutralMode(NeutralMode.Brake);
  m_clamp.setNeutralMode(NeutralMode.Brake);
  

   // For encoder
   //I have no idea how to make one for SparkMax
  m_left_wheel.setSelectedSensorPosition(0);
  m_right_wheel.setSelectedSensorPosition(0);

    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
  m_arm.setOpenLoopRampRate(0.4); //no idea if this one works
  m_left_wheel.configOpenloopRamp(0.4); 
  m_right_wheel.configOpenloopRamp(0.4); // may need change in the future when robot is active
    // public void ArmPlace(double ) {
      //Unused for now
    //}
  public void Intake(double Direction){
    leftHand.set(Direction);
    rightHand.set(Direction * -1);
  }
  
  if (headingReady) {
    if (i==10) {
        System.out.println(beginPID(currentHeading, 30));
        i=0;
    }
    else {
        i++;
    }
}

  public PIDController getPID() {
    return PID;
  }

  private double beginPID(double currentMeasurement, double goalPoint) {
      return PID.calculate(currentMeasurement, goalPoint);
  }
}

/**Changes needed but can't be done rn:
 * 
 * Figure out the motors for the clamp and wheels.
 * Install the new libraries for each new type of motor.
 * Use the motor types for the braking, Accel, etc.
 *
 * Add a following version of m_arm when for each new arm motor.
 * 
*/
