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

public class MotorController extends SubsystemBase{
  private CANSparkMax LeftHand;
  private CANSparkMax RightHand;
  private CANSparkMax Arm;
  private CANSparkMax Arm2;
  RelativeEncoder ArmEncoder; 
  private SparkMaxPIDController PID; 
  private static double AkP;
  private static double AkI;
  private static double AkD;
  private double AkIz;
  private double AkFF;
  private double AkMaxOutput;
  private double AkMinOutput;
  
  public MotorController() {
      // Big Arm Motor
  Arm = new CANSparkMax(Constants.MOTOR_ARM_1, MotorType.kBrushless);
  ArmEncoder = Arm.getEncoder();
  Arm.restoreFactoryDefaults();
  PID = Arm.getPIDController();

  // PID coefficients
  AkP = 0.05; 
  AkI = .02;
  AkD = 0; 
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

  Arm2 = new CANSparkMax(Constants.MOTOR_ARM_2, MotorType.kBrushless);
  Arm2.follow(Arm);

  // Controls wheels that suck em up
  LeftHand = new CANSparkMax(Constants.MOTOR_LEFT_GRIP, MotorType.kBrushless);
  RightHand = new CANSparkMax(Constants.MOTOR_RIGHT_GRIP, MotorType.kBrushless);
  RightHand.setInverted(true);


  Arm.setIdleMode(IdleMode.kBrake);
  Arm2.setIdleMode(IdleMode.kBrake);
  LeftHand.setIdleMode(IdleMode.kBrake);
  RightHand.setIdleMode(IdleMode.kBrake);


    // limits acceleration, takes 0.4 seconds to accelerate from 0 to 100%
  Arm.setOpenLoopRampRate(0.4); 
  LeftHand.setOpenLoopRampRate(0.1); 
  RightHand.setOpenLoopRampRate(0.1); 

  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
      SmartDashboard.putNumber("Arm Angle", ArmEncoder.getPosition());
  }
  
  public void Intake(double Direction){
    LeftHand.set(Direction);
    RightHand.set(Direction);
  }

  public SparkMaxPIDController getArmPID() {
    return PID;
  }
  public void ArmPID(double goalPoint) {
    ProfiledPIDController Equation = new ProfiledPIDController(AkP, AkI, AkD, new TrapezoidProfile.Constraints(300, 150));
    Arm.set(MathUtil.clamp(Equation.calculate(goalPoint), -0.25, 0.25));
  } 
  public void ArmMove(double Movement){
    Arm.set(Movement);
  }
}
