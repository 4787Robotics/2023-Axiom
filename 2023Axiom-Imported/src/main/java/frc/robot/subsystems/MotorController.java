package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.security.PublicKey;

import org.ejml.equation.Equation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;

import edu.wpi.first.wpilibj.Timer;

public class MotorController extends SubsystemBase{
  ProfiledPIDController Equation = new ProfiledPIDController(AkP, AkI, AkD, new TrapezoidProfile.Constraints(300, 150));
  public CANSparkMax LeftHand;
  public CANSparkMax RightHand;
  public CANSparkMax Arm;
  private CANSparkMax Arm2;
  public WPI_TalonSRX HandUpDown;
  public CANSparkMax ArmHolder;
  public RelativeEncoder ArmEncoder; 
  public RelativeEncoder LeftEncoder; 
  public RelativeEncoder RightEncoder; 
  private double LeftStartingPos;
  private double RightStartingPos;
  //private SparkMaxPIDController PID; 
  private static double AkP;
  private static double AkI;
  private static double AkD;
  private double AkIz;
  private double AkFF;
  private double AkMaxOutput;
  private double AkMinOutput;
  private int currentPointNum = 0; //doesn't matter what this is as long as it is more than 2
  
  private TalonSRX electro_test = new TalonSRX(9);


  public MotorController() {
    // Big Arm Motor
  Arm = new CANSparkMax(Constants.MOTOR_ARM_1, MotorType.kBrushless);
  ArmEncoder = Arm.getEncoder();
  //PID = Arm.getPIDController();

  

  // PID coefficients
  AkP = 0.04; 
  AkI = 0.005;
  AkD = 0; 
  AkIz = 0; 
  AkFF = 0; 
  AkMaxOutput = 1; 
  AkMinOutput = -1;

  /*  set PID coefficients
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
  double p = SmartDashboard.getNumber("P Gain", .05);
  double i = SmartDashboard.getNumber("I Gain", 0);
  double d = SmartDashboard.getNumber("D Gain", 0.0);
  double iz = SmartDashboard.getNumber("I Zone", 0);
  double ff = SmartDashboard.getNumber("Feed Forward", 0);
  double max = SmartDashboard.getNumber("Max Output", 0);
  double min = SmartDashboard.getNumber("Min Output", 0);
  double rotations = SmartDashboard.getNumber("Set Rotations", 0);

  //if PID coefficients on SmartDashboard have changed, write new values to controller
  if((p != AkP)) { PID.setP(p); AkP = p; }
  if((i != AkI)) { PID.setI(i); AkI = i; }
  if((d != AkD)) { PID.setD(d); AkD = d; }
  if((iz != AkIz)) { PID.setIZone(iz); AkIz = iz; }
  if((ff != AkFF)) { PID.setFF(ff); AkFF = ff; }
  if((max != AkMaxOutput) || (min != AkMinOutput)) { 
   PID.setOutputRange(min, max); 
   AkMinOutput = min; AkMaxOutput = max; 
  } */

  //PID.setReference(rotations, CANSparkMax.ControlType.kPosition);
  

  Arm2 = new CANSparkMax(Constants.MOTOR_ARM_2, MotorType.kBrushless);
  Arm2.follow(Arm);

  // Controls wheels that suck em up
  LeftHand = new CANSparkMax(Constants.MOTOR_LEFT_GRIP, MotorType.kBrushless);
  LeftHand.setInverted(true);
  LeftEncoder = LeftHand.getEncoder();
  LeftStartingPos = LeftEncoder.getPosition();
  RightHand = new CANSparkMax(Constants.MOTOR_RIGHT_GRIP, MotorType.kBrushless);
  RightHand.setInverted(false);
  RightEncoder = RightHand.getEncoder();
  RightStartingPos = RightEncoder.getPosition();

  Arm.setIdleMode(IdleMode.kBrake);
  Arm2.setIdleMode(IdleMode.kBrake);
  LeftHand.setIdleMode(IdleMode.kBrake);
  RightHand.setIdleMode(IdleMode.kBrake);


    // limits acceleration, takes 0.2 seconds to accelerate from 0 to 100%
  Arm.setOpenLoopRampRate(0.25); 
  Arm2.setOpenLoopRampRate(0.25); 
  LeftHand.setOpenLoopRampRate(0.2); 
  RightHand.setOpenLoopRampRate(0.2); 

  

  HandUpDown = new WPI_TalonSRX(Constants.MOTOR_MOVE_GRIP);
  HandUpDown.enableVoltageCompensation(true);
  HandUpDown.setInverted(true); 
  HandUpDown.setNeutralMode(NeutralMode.Brake);
  HandUpDown.configPeakCurrentLimit(15, 10);
  HandUpDown.configPeakCurrentDuration(100, 10);
  HandUpDown.configContinuousCurrentLimit(20, 10);

  ArmHolder = new CANSparkMax(Constants.MOTOR_ARM_HOLD, MotorType.kBrushless);
  ArmHolder.setOpenLoopRampRate(1);
  ArmHolder.setInverted(false);
  ArmHolder.setIdleMode(IdleMode.kBrake);

  Arm.setSmartCurrentLimit(35);
  Arm2.setSmartCurrentLimit(35);
  LeftHand.setSmartCurrentLimit(35);
  RightHand.setSmartCurrentLimit(35);
  ArmHolder.setSmartCurrentLimit(20);
  
  
  }

  

  @Override
  public void periodic() {
      SmartDashboard.putNumber("Arm Angle", ArmEncoder.getPosition());
      SmartDashboard.putNumber("LeftHand Angle", LeftEncoder.getPosition());
      SmartDashboard.putNumber("RightHand Angle", RightEncoder.getPosition());
      SmartDashboard.putNumber("LeftArm Angle", LeftEncoder.getPosition());
      SmartDashboard.putNumber("RightArm Angle", RightEncoder.getPosition());
    }
  
  public void Intake(double Direction){
    LeftHand.set(Direction);
    RightHand.set(Direction);
  }

  public void LeftHandMove(double Direction, boolean resetPOS){
    if (resetPOS && Direction > .025){ //Will slow until reaching starting position
      Direction = MathUtil.clamp(Direction, 0, .65);
    }
    LeftHand.set(Direction);  
  }

  public void RightHandMove(double Direction, boolean resetPOS){
    if (resetPOS && Direction > .025){ //Will slow until reaching starting position
      Direction = MathUtil.clamp(Direction, 0, .65);
    }
    RightHand.set(Direction);
  }
  
  public void ArmPID(double goalPoint, int newPointNum) {
    if (currentPointNum != newPointNum){
      currentPointNum = newPointNum;
      Equation = new ProfiledPIDController(AkP, AkI, AkD, new TrapezoidProfile.Constraints(185, 80));
      Equation.reset(ArmEncoder.getPosition());
    }
    if (goalPoint == 0 && (ArmEncoder.getPosition() < 8)) {
      Arm.set(MathUtil.clamp(Equation.calculate(ArmEncoder.getPosition(), goalPoint), -0.075, .4) - 0.3);
    } else {
      Arm.set(MathUtil.clamp(Equation.calculate(ArmEncoder.getPosition(), goalPoint), -0.075, .4));
    } 
  }

  public void ArmMove(double Movement){
    Arm.set(Movement/2.5); //Change this for more motor power
  }

  public void GripMove(double UpDown){
    HandUpDown.set(UpDown);
  }

  public void ArmHolderStart(double speed){
      ArmHolder.set(speed);
  }
}
