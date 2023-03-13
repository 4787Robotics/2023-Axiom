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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class MoveTo extends CommandBase {
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  private RamseteCommand ramseteCommand;

  public MoveTo() {
      addRequirements(RobotContainer.m_driveTrain);
  }

  /**
   * Changes the RamseteCommand
   * @param m_driveTrain
   * @param m_config
   * @param TurnTo
   */

  public void MovingRamseteCommand(DriveTrain m_driveTrain, TrajectoryConfig m_config, double MoveTo) {
      driveTrain = m_driveTrain;
      addRequirements(driveTrain);

      trajectory = TrajectoryGenerator.generateTrajectory(
              //start
              new Pose2d(0,0, new Rotation2d(0)),
              //turn
              List.of(new Translation2d(0,0)),
              //end
              new Pose2d(MoveTo, 0, new Rotation2d(0)), //Is X or Y the one I want to change???
              // Pass config
              m_config
      );

      driveTrain.resetOdometry(trajectory.getInitialPose());

      ramseteCommand = new RamseteCommand(trajectory, 
          driveTrain::getPose,
          new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_A),
          new SimpleMotorFeedforward(
            Constants.KS_VOLTS,
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

      ramseteCommand.andThen(() -> driveTrain.driveRobot(false, 0, 0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
