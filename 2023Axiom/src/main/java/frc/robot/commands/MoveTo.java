// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class MoveTo extends CommandBase {
  /** Creates a new TurnAngle. */
  private DriveTrain driveTrain;
  public Trajectory trajectory;
  private RamseteCommand ramseteCommand;

  public MoveTo() {addRequirements(RobotContainer.m_driveTrain);}

  public Command changeRamseteCommand(DriveTrain m_driveTrain, double meters) {
    driveTrain = m_driveTrain;
    addRequirements(driveTrain);

    // im not sure what .addConstraint is, so we will figure about that.
    DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.KS_VOLTS,
                                   Constants.KV_VOLT_SECONDS_PER_METER,
                                   Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
        Constants.K_DRIVE_KINEMATICS,
        4
    );
    TrajectoryConfig config =
      new TrajectoryConfig(Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(Constants.K_DRIVE_KINEMATICS) //ensures max speed is actually obeyed
        .addConstraint(autoVoltageConstraint)
        .setReversed(false); //voltage constraint

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0,new Rotation2d(0)), 
      List.of(new Translation2d(meters-1, 0)), // im not sure about this
      new Pose2d(meters, 0, new Rotation2d(0)),
      config
    );

    ramseteCommand = new RamseteCommand(trajectory, 
      driveTrain::getPose,
      new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_A),
      new SimpleMotorFeedforward(Constants.KS_VOLTS,
                                Constants.KV_VOLT_SECONDS_PER_METER,
                                Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
      Constants.K_DRIVE_KINEMATICS,
      driveTrain::getWheelSpeeds,
      new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
      new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrain::tankDriveVolts,
      driveTrain
      );

    driveTrain.resetOdometry(trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> driveTrain.driveRobot(false, 0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
    // RammseteAutonomousCommand.getRammseteAutonomousCommand(driveTrain, 18);
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
