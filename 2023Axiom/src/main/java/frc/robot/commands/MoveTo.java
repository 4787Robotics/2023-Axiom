// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class MoveTo extends CommandBase {
  /** Creates a new TurnAngle. */
  private final DriveTrain driveTrain;
  private double meters; // positive meters = forward, while negative meters = backward
  public Trajectory moveToTrajectory;

  public MoveTo(DriveTrain m_driveTrain, double m_meters) {
    driveTrain = m_driveTrain;
    meters = m_meters;

    // im not sure what .addConstraint is, so we will figure about that.
        
    moveToTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(new Translation2d(0, 0), new Translation2d(meters,0)), // im not sure about this
      new Pose2d(meters, 0, new Rotation2d(0)),
      RammseteAutonomousCommand.config
    );

    Robot.trajectoryArray[17] = moveToTrajectory;

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
    /*
    if (meters < DriveTrain.m_right1.getSelectedSensorPosition()){
      driveTrain.drive.arcadeDrive(0.5, 0);
    }
    if (meters > DriveTrain.m_right1.getSelectedSensorPosition()){
      driveTrain.drive.arcadeDrive(0, 0);
      driveTrain.resetEncoders();
    }
    */
    RammseteAutonomousCommand.getRammseteAutonomousCommand(driveTrain, 18);
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
