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
import frc.robot.CXbox;
import frc.robot.Constants;
import frc.robot.subsystems.MotorController;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
/** An example command that uses an example subsystem. */
public class ArmCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorController m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmCommand(MotorController subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
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
      System.out.println("Two or more buttons are pressed");
    } else if(CXbox.XboxADown()) {
      m_subsystem.ArmPID(1,0);//Lowest Point, Grounded
    } else if (CXbox.XboxBDown()) {
      m_subsystem.ArmPID(1,66.2113);//Mid Point, from below
    } else if (CXbox.XboxYDown()) {
      m_subsystem.ArmPID(1,83.725);//Highmid point?
    } else if (CXbox.XboxXDown()) {
      m_subsystem.ArmPID(1, -7.5);//Highmid point?
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