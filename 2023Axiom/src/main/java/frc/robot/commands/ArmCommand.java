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
    ArmEncoderC = m_subsystem.ArmEncoder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int Pushed = 0;

    if (ArmEncoderC.getPosition() > 120) {
      m_subsystem.ArmMove(-1);
    } else {
      if (CXbox.XboxADown()){
        Pushed++;
      }
      if (CXbox.XboxYDown()){
        Pushed++;
      }
      if (CXbox.XboxXDown()){
        Pushed++;
      }
      if (CJoystick.joystickButton8Down()){
        Pushed++;
      }
      if (CJoystick.joystickButton10Down()){
        Pushed++;
      }
      if (CJoystick.joystickButton12Down()){
        Pushed++;
      }//Used to stop motor
      if(CXbox.XboxADown() || CJoystick.joystickButton12Down()) {
        m_subsystem.ArmPID(Constants.LOW_LEVEL, 0);//Lowest Point, Grounded
      } else if (CXbox.XboxBDown() || CJoystick.joystickButton10Down()) {
        m_subsystem.ArmPID(Constants.MID_LEVEL, 1);//Mid point
      } else if (CXbox.XboxYDown() || CJoystick.joystickButton8Down()) {
        m_subsystem.ArmPID(Constants.HIGH_LEVEL,2 );//Highmid point?
      } else if (CJoystick.getJoystickThrottle() > .5) {
        m_subsystem.ArmMove((CJoystick.getJoystickThrottle() - 0.25));
      } else if (CJoystick.getJoystickThrottle() < -.5) {
        m_subsystem.ArmMove((CJoystick.getJoystickThrottle() + 0.25));
      } else {
        m_subsystem.ArmMove(0);
      }
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