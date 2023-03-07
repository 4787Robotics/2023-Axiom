//Commenting out autonomous to test Teleop
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.MotorSafety;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RammseteAutonomousCommand extends CommandBase{
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  public DriveTrain driveTrain;
  Pose2d initialPose;
  public TrajectoryConfig config;
  Trajectory trajectory = new Trajectory();

  /**
  * Creates a new RammseteAutonomousCommand.
  *
  * @param subsystem The subsystem used by this command.
  * @param pathNumber The number that decides the path the robot is going to follow
   * @return the path and then stops the robot
  */
  public Command RammseteAutonomousCommand(DriveTrain subsystem, int pathNumber) {
    if (pathNumber == 1) {
      trajectory = Robot.trajectoryArray.get(0);
    }
    if (pathNumber == 2) {
      trajectory = Robot.trajectoryArray.get(1);
    }
    if (pathNumber == 3) {
      trajectory = Robot.trajectoryArray.get(2);
    }
    if (pathNumber == 4) {
      trajectory = Robot.trajectoryArray.get(3);
    }
    if (pathNumber == 5) {
      trajectory = Robot.trajectoryArray.get(4);
    }
    if (pathNumber == 6) {
      trajectory = Robot.trajectoryArray.get(5);
    }
    if (pathNumber == 7) {
      trajectory = Robot.trajectoryArray.get(6);
    }
    if (pathNumber == 8) {
      trajectory = Robot.trajectoryArray.get(7);
    }
    if (pathNumber == 9) {
      trajectory = Robot.trajectoryArray.get(8);
    }
    if (pathNumber == 10) {
      trajectory = Robot.trajectoryArray.get(9);
    }
    if (pathNumber == 11) {
      trajectory = Robot.trajectoryArray.get(10);
    }
    driveTrain = RobotContainer.m_driveTrain;
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

    initialPose = trajectory.getInitialPose();

    RamseteCommand autonomousCommand = new RamseteCommand(
      trajectory,
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
    return autonomousCommand.andThen(() -> driveTrain.tankDriveVolts(0,0));
}
public void resetOdometryInitialPose() {
  driveTrain.resetOdometry(initialPose);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

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

/*
put all the paths in constants
assign a number
create if statements of 
*/