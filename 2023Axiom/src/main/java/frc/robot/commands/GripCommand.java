// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.CXbox;
import frc.robot.CJoystick;
import frc.robot.Constants;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
/** An example command that uses an example subsystem. */
public class GripCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorController m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GripCommand(MotorController subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies, if any.
    //addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  private char level;
  public void execute() {
    if (CXbox.getLeftTriggerWithDeadzone() > 0 && CXbox.getRightTriggerWithDeadzone() > 0){
      System.out.println("Two or more buttons are pressed");
      m_subsystem.intake(0); //Don't move
    } else if(CXbox.getRightTriggerWithDeadzone() > 0) {
      m_subsystem.intake(0.5); //Pull in
    } else if (CXbox.getLeftTriggerWithDeadzone() > 0) {
      m_subsystem.intake(-0.5); //Pull out
    } else{
      m_subsystem.intake(0); //Don't move
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