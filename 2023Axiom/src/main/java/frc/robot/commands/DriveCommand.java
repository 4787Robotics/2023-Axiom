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

public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;
  XboxController controller;
  private Spark m_left1, m_right2;
  private Spark m_left2, m_right1;
  DifferentialDrive drive;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveTrain subsystem, XboxController inputController) {
        controller = inputController;
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
    if(CXbox.getRightStickXWithDeadzone() > Constants.rightStickDeadzone) {
      m_subsystem.turnRight(controller.getRightX());
    } else if (CXbox.getRightStickXWithDeadzone() < -Constants.rightStickDeadzone) {
      m_subsystem.turnLeft(controller.getRightX());
    }
    if(CXbox.getLeftStickYWithDeadzone() < -Constants.leftStickDeadzone) {
      m_subsystem.moveForward(controller.getLeftY());
    } else if (CXbox.getRightStickYWithDeadzone() > Constants.rightStickDeadzone) {
      m_subsystem.moveBackward(controller.getLeftY());
    }
    else {
      feed();
    }                                                                                                                                 
  }

  private void feed() {
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
