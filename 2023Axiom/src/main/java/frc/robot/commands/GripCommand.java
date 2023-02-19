// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.CJoystick;
import frc.robot.CXbox;
import frc.robot.Constants;
import frc.robot.subsystems.MotorController;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

/** An example command that uses an example subsystem. */
public class GripCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorController m_subsystem;
  CXbox CXbox = new CXbox();
  CJoystick CJoystick = new CJoystick();
  RelativeEncoder GripEncoder; 
  private CANSparkMax Grip;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GripCommand(MotorController subsystem) {
    m_subsystem = subsystem;
    Grip = new CANSparkMax(Constants.MOTOR_LEFT_GRIP, MotorType.kBrushless); 
    GripEncoder = Grip.getEncoder(); //The Encoder only checks the left motor
    Grip.restoreFactoryDefaults();  //This won't matter because the right motor will copy movements anyway
    // Use addRequirements() here to declare subsystem dependencies, if any.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (CXbox.getLeftTriggerWithDeadzone() > 0 && CXbox.getRightTriggerWithDeadzone() > 0){
      System.out.println("Implosion upcoming");
      m_subsystem.Intake(0); //Don't move
    } else if(CXbox.getRightTriggerWithDeadzone() > 0 && GripEncoder.getPosition() < 10) {
      m_subsystem.Intake(0.1); //Pull in
    } else if (CXbox.getLeftTriggerWithDeadzone() > 0 && GripEncoder.getPosition() > -60){
      m_subsystem.Intake(-0.1); //Pull out
    } else {
      m_subsystem.Intake(0); //Don't move
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