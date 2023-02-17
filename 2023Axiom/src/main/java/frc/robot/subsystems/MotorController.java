package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.security.PublicKey;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MotorController extends SubsystemBase{
  private CANSparkMax LeftHand;
  private CANSparkMax RightHand;
  private CANSparkMax Arm;
  private CANSparkMax Arm2;
  RelativeEncoder ArmEncoder; 
  private SparkMaxPIDController PID; 
  private double AkP, AkI, AkD, AkIz, AkFF, AkMaxOutput, AkMinOutput;
  
  public MotorController() {
      // Big Arm Motor
  Arm = new CANSparkMax(Constants.MOTOR_ARM_1, MotorType.kBrushless);
  Arm2 = new CANSparkMax(Constants.MOTOR_ARM_2, MotorType.kBrushless);
  Arm2.follow(Arm);
  Arm.restoreFactoryDefaults();
  ArmEncoder = Arm.getEncoder();
  PID = Arm.getPIDController();

  // PID coefficients
  AkP = 0.1; 
  AkI = 1e-4;
  AkD = 1; 
  AkIz = 0; 
  AkFF = 0; 
  AkMaxOutput = 1; 
  AkMinOutput = -1;

  // set PID coefficients
  PID.setP(AkP);
  PID.setI(AkI);
  PID.setD(AkD);
  PID.setIZone(AkIz);
  PID.setFF(AkFF);
  PID.setOutputRange(AkMinOutput, AkMaxOutput);

  // display PID coefficients on SmartDashboard
  SmartDashboard.putNumber("P Gain", AkP);
  SmartDashboard.putNumber("I Gain", AkI);
  SmartDashboard.putNumber("D Gain", AkD);
  SmartDashboard.putNumber("I Zone", AkIz);
  SmartDashboard.putNumber("Feed Forward", AkFF);
  SmartDashboard.putNumber("Max Output", AkMaxOutput);
  SmartDashboard.putNumber("Min Output", AkMinOutput);
  SmartDashboard.putNumber("Set Rotations", 0);

  // read PID coefficients from SmartDashboard
  double p = SmartDashboard.getNumber("P Gain", 0);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double rotations = SmartDashboard.getNumber("Set Rotations", 0);

  // if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != AkP)) { PID.setP(p); AkP = p; }
  if((i != AkI)) { PID.setI(i); AkI = i; }
  if((d != AkD)) { PID.setD(d); AkD = d; }
  if((iz != AkIz)) { PID.setIZone(iz); AkIz = iz; }
  if((ff != AkFF)) { PID.setFF(ff); AkFF = ff; }
  if((max != AkMaxOutput) || (min != AkMinOutput)) { 
    PID.setOutputRange(min, max); 
    AkMinOutput = min; AkMaxOutput = max; 
  }

  PID.setReference(rotations, CANSparkMax.ControlType.kPosition);
    
  SmartDashboard.putNumber("SetPoint", rotations);
  SmartDashboard.putNumber("ProcessVariable", ArmEncoder.getPosition());

  // Controls wheels that suck em up
  LeftHand = new CANSparkMax(Constants.MOTOR_LEFT_HAND, MotorType.kBrushless);
  RightHand = new CANSparkMax(Constants.MOTOR_RIGHT_HAND, MotorType.kBrushless);
  LeftHand.setInverted(true); 
// ewqeKeeps the motors in place and stops them frowm moving without input
  Arm.setIdleMode(IdleMode.kBrake);
  Arm2.setIdleMode(IdleMode.kBrake);
  LeftHand.setIdleMode(IdleMode.kBrake);
  RightHand.setIdleMode(IdleMode.kBrake);


    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
  Arm.setOpenLoopRampRate(0.4); 
  LeftHand.setOpenLoopRampRate(0.1); 
  RightHand.setOpenLoopRampRate(0.1); 

  Arm.set(.25); //changes top speed. stay between -1 and 1 for safety
  LeftHand.set(.25);
  RightHand.set(.25);
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
      SmartDashboard.putNumber("Arm Angle", ArmEncoder.getPosition());
  }
  
  public void Intake(double Direction){
    //add inverts if going backward
    LeftHand.setVoltage(Direction);
    RightHand.setVoltage(Direction);
  }

  public PIDController getArmPID() {
    return PID;
  }
  public double ArmPID(double currentMeasurement, double goalPoint) {
    Arm.set(PID.calculate(ArmEncoder.getPosition(), goalPoint));
  }
  public void ArmMove(double Movement){
    Arm.set(Movement);
  }
}