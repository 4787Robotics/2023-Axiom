package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.CXbox;
import frc.robot.Constants;
import frc.robot.CJoystick;
import frc.robot.subsystems.MotorController;
import edu.wpi.first.wpilibj.Timer;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;
import javax.swing.text.Position;
import edu.wpi.first.math.MathUtil;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.RelativeEncoder;
/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorController m_subsystem;
  RelativeEncoder ArmEncoderC; 
  CXbox CXbox = new CXbox();
  CJoystick CJoystick = new CJoystick();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(MotorController subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    CANSparkMax ArmC = new CANSparkMax(Constants.MOTOR_ARM_1, MotorType.kBrushless);
    ArmC.restoreFactoryDefaults();
    RelativeEncoder ArmEncoderC = ArmC.getEncoder();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int Pushed = 0;
    if (CXbox.XboxBDown() == true){
      Pushed++;
    }
    if (CXbox.XboxADown() == true){
      Pushed++;
    }
    if (CXbox.XboxYDown() == true){
      Pushed++;
    }
    if (CXbox.XboxXDown() == true){
      Pushed++;
    }
    if (Pushed >= 2){
      System.out.println("Two or more buttons are pressed"); //I may want to change letter buttons to user prefrence
    } else if (CXbox.XboxBDown() || CJoystick.joystickButton1Down()) {
      m_subsystem.ArmPID(ArmEncoderC.getPosition()); //Tells the PID to go to the spot that it is it, hopefully stopping it in place.
    } else if(CXbox.XboxADown() || CJoystick.joystickButton12Down()) {
      while (Math.abs(ArmEncoderC.getPosition()) > 1) {
        m_subsystem.ArmPID(Constants.LOW_LEVEL);//Lowest Point, Grounded
      }
    } else if (CXbox.XboxBDown() || CJoystick.joystickButton10Down()) {
      while (Math.abs(ArmEncoderC.getPosition() - Constants.MID_LEVEL) > 1) {
        m_subsystem.ArmPID(Constants.MID_LEVEL);//Mid point
      }
    } else if (CXbox.XboxYDown() || CJoystick.joystickButton8Down()) {
      while (Math.abs(ArmEncoderC.getPosition() - Constants.HIGH_LEVEL) > 1) {
        m_subsystem.ArmPID(Constants.HIGH_LEVEL);//Highmid point?
      }
    } else if (CJoystick.getJoystickThrottle() > .5) {
      m_subsystem.ArmMove((CJoystick.getJoystickThrottle() - 0.25));
    } else if (CJoystick.getJoystickThrottle() < -.5) {
      m_subsystem.ArmMove((CJoystick.getJoystickThrottle() + 0.25));
    } else {
      m_subsystem.ArmMove(0);
    }
  } //I have no idea what to put here for PID values and such

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}