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
import javax.swing.GrayFilter;
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
  private RelativeEncoder ArmEncoderC; 
  private RelativeEncoder LeftEncoderC;
  private RelativeEncoder RightEncoderC;
  private double LeftStartingPos;
  private double RightStartingPos;
  private final CXbox m_cXbox;
  private final CJoystick m_cJoystick;
  private boolean gripPlace = false;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(MotorController subsystem, CXbox ArmCXbox, CJoystick ArmCJoystick) {
    m_subsystem = subsystem;
    m_cXbox = ArmCXbox;
    m_cJoystick = ArmCJoystick;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmEncoderC = m_subsystem.ArmEncoder;
    LeftEncoderC = m_subsystem.LeftEncoder;
    RightEncoderC = m_subsystem.RightEncoder;
    LeftStartingPos = LeftEncoderC.getPosition();
    RightStartingPos = RightEncoderC.getPosition();
  }

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

    m_subsystem.ArmMove((m_cJoystick.getJoystickYWithDeadzone()));

    if(m_cJoystick.joystickButton1Down() == true && m_cJoystick.joystickButton2Down() == true) { //will change for user
      m_subsystem.Intake(-0.1); //Pull out
    } else if (m_cJoystick.joystickButton1Down() == true) { 
      m_subsystem.Intake(0.1); //Pull in
    } else if (m_cJoystick.joystickButton3Down()) {
      m_subsystem.LeftHandMove(-0.1);
    } else if (m_cJoystick.joystickButton4Down()) {
      m_subsystem.RightHandMove(-0.1);
    } else {
      m_subsystem.Intake(0); //Don't move
    }

    if(m_cJoystick.getJoystickThrottle() > .8 && gripPlace == true){
      m_subsystem.GripMove(-.2);
      gripPlace = false;
    } else if (m_cJoystick.getJoystickThrottle() < -.8 && gripPlace == false){
      m_subsystem.GripMove(.2);
      gripPlace = false;
    } else {
      m_subsystem.GripMove(0);
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
