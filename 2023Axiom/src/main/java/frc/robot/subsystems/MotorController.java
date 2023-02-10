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
  private RelativeEncoder armEncoder; 
  private PIDController PID; {
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
  Arm.setOpenLoopRampRate(0.4); 
  LeftHand.setOpenLoopRampRate(0.1); 
  RightHand.setOpenLoopRampRate(0.1); 

  Arm.set(.5); //changes top speed. stay between -1 and 1 for safety
  LeftHand.set(.5);
  RightHand.set(.5);

  }// may need change in the future when robot is active
  
  public void Intake(double Direction){
    //add inverts if going backward
    LeftHand.setVoltage(Direction);
    RightHand.setVoltage(Direction);
  }

  public PIDController getArmPID() {
    return PID;
  }
  public double ArmPID(double currentMeasurement, double goalPoint) {
    return PID.calculate(currentMeasurement, goalPoint);
  }
  public void ArmMove(double Movement){
    Arm.set(Movement);
  }
}