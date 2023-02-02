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
  private CANSparkMax LeftHand;
  private CANSparkMax RightHand;
  private CANSparkMax Arm;
  private RelativeEncoder armEncoder; {
  //private PIDController PID;

      // Big Arm Motor
  Arm = new CANSparkMax(Constants.MOTOR_ARM, MotorType.kBrushless);
  armEncoder = Arm.getEncoder(); //for PID
 // PID = new PIDController(1, 1, 1); //CHANGE BEFORE TEST

  // Controls wheels that suck em up
  LeftHand = new CANSparkMax(Constants.MOTOR_LEFT_HAND, MotorType.kBrushless);
  RightHand = new CANSparkMax(Constants.MOTOR_RIGHT_HAND, MotorType.kBrushless);

  // Makes left motor go the opposite of the right motor
  LeftHand.setInverted(true); 
   // ewqeKeeps the motors in place and stops them frowm moving without input
  Arm.setIdleMode(IdleMode.kBrake);
  LeftHand.setIdleMode(IdleMode.kBrake);
  RightHand.setIdleMode(IdleMode.kBrake);


    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
  Arm.setOpenLoopRampRate(0.4); //no idea if this one works
  LeftHand.setOpenLoopRampRate(0.1); 
  RightHand.setOpenLoopRampRate(0.1);} // may need change in the future when robot is active

  public void Intake(double Direction){
    LeftHand.set(Direction);
    RightHand.set(Direction * -1);
  }
/*
  if (headingReady) {
    int i;
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
  } */
} @

/**Changes needed but can't be done rn:
 * 
 * Figure out the motors for the clamp and wheels.
 * Install the new libraries for each new type of motor.
 * Use the motor types for the braking, Accel, etc.
 *
 * Add a following version of m_arm when for each new arm motor.
 * 
*/
