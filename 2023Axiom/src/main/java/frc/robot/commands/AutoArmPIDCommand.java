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
public class AutoArmPIDCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorController m_subsystem;
  private RelativeEncoder ArmEncoderC; 
  public int level;
  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutoArmPIDCommand(MotorController subsystem) {
    m_subsystem = subsystem;
      
    


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmEncoderC = m_subsystem.ArmEncoder;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(level == 0) { 
        m_subsystem.ArmPID(Constants.LOW_LEVEL, 0); //Low Point AKA Grounded
    } else if (level == 1) {
        m_subsystem.ArmPID(Constants.MID_LEVEL, 1); //Mid point
    } else if (level == 2) {
        m_subsystem.ArmPID(Constants.HIGH_LEVEL,2); //High point
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } 
}
