// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class MoveTo extends CommandBase {
  /** Creates a new TurnAngle. */
  private final DriveTrain driveTrain;
  private double meters; // positive meters = forward while negative meters = backward

  public MoveTo(DriveTrain m_driveTrain, double meters) {
    driveTrain = m_driveTrain;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (meters < DriveTrain.m_right1.getSelectedSensorPosition()){
      driveTrain.drive.arcadeDrive(0.5, 0);
    }
    if (meters > DriveTrain.m_right1.getSelectedSensorPosition()){
      driveTrain.resetEncoders();
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
