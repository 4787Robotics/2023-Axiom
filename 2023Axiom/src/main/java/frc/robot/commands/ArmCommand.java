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
  private final CXbox m_cxbox;
  private final CJoystick ArmCJoystick;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(MotorController subsystem, CXbox ArmCXbox, CJoystick m_cjoystick) {
    m_subsystem = subsystem;
    m_cxbox = ArmCXbox;
    ArmCJoystick = m_cjoystick;

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

    // if (ArmEncoderC.getPosition() > 4) {
    //   m_subsystem.ArmPID(Constants.HIGH_LEVEL, 2);
    // } else {
    //   if( ArmCJoystick.joystickButton12Down()) { 
    //     m_subsystem.ArmPID(Constants.LOW_LEVEL, 0);//Low Point AKA Grounded
    //   } else if ( ArmCJoystick.joystickButton10Down()) {
    //     m_subsystem.ArmPID(Constants.MID_LEVEL, 1);//Mid point
    //   } else if ( ArmCJoystick.joystickButton8Down()) {
    //     m_subsystem.ArmPID(Constants.HIGH_LEVEL,2 );//High point
    //   } else 

    boolean armMove = false;
    if (ArmCJoystick.getJoystickThrottle() > .5) {
      m_subsystem.ArmMove((ArmCJoystick.getJoystickYWithDeadzone()));
      armMove = true;
    } else if (ArmCJoystick.getJoystickThrottle() < -.5) {
      m_subsystem.ArmMove((ArmCJoystick.getJoystickYWithDeadzone()));
      armMove = true;
    } else {
      m_subsystem.ArmMove(0);
    }
    if(ArmCJoystick.joystickButton1Down() == true && ArmCJoystick.joystickButton2Down() == true) {
      m_subsystem.Intake(-0.1); //Pull out
      // if (!armMove) {
      //   m_subsystem.ArmMove(0.1);
      // }
    } else if (ArmCJoystick.joystickButton1Down() == true) {
      m_subsystem.Intake(0.1); //Pull out
      // if (!armMove) {
      //   m_subsystem.ArmMove(0.1);
      // }
    } else {
      m_subsystem.Intake(0); //Don't move
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
