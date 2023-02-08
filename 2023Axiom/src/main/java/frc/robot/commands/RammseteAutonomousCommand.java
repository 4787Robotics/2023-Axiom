//Commenting out autonomous to test Teleop
package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.MotorSafety;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;


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
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class RammseteAutonomousCommand extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public final DriveTrain driveTrain;
    Pose2d initialPose;
    public TrajectoryConfig config;
    public Trajectory exampleTrajectory;
    /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public RammseteAutonomousCommand(DriveTrain subsystem) {
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
  }
    
  public Command createAutonomousCommand(Trajectory trajectory) {
        
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
      driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
      //wpiLIB has it as a return function because they're not using a void function to call the autonomous command
      return autonomousCommand.andThen(() -> driveTrain.tankDriveVolts(0,0));
    

}
public void resetOdometryInitialPose() {
  driveTrain.resetOdometry(initialPose);
}


    // An ExampleCommand will run in autonomous  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      final DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(Constants.KS_VOLTS, Constants.KV_VOLT_SECONDS_PER_METER, Constants.KA_VOLT_SECONDS_SQUARED_PER_METER), 
          Constants.K_DRIVE_KINEMATICS,10); 
    
        TrajectoryConfig config = 
          new TrajectoryConfig(Constants.K_MAX_SPEED_METERS_PER_SECOND, Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
          .setKinematics(Constants.K_DRIVE_KINEMATICS)
          .addConstraint(autoVoltageConstraint);
    
        Trajectory exampleTrajectory = 
          TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), 
          List.of(new Translation2d(0,1)),
          new Pose2d(0, 0, new Rotation2d(0)), 
          config);
      // COMMENTED OUT BC NON FUNCTIONAL:
      //   RamseteCommand ramseteCommand = new RamseteCommand(
      //     exampleTrajectory,
      //     driveTrain::getPose,
      //     new RamseteController(Constants.K_RAMSETE_B, Constants.K_RAMSETE_A),
      //     new SimpleMotorFeedforward(Constants.KS_VOLTS,
      //                                 Constants.KV_VOLT_SECONDS_PER_METER,
      //                                 Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
      //     Constants.K_DRIVE_KINEMATICS,
      //     driveTrain::getWheelSpeeds,
      //     new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
      //     new PIDController(Constants.KP_DRIVE_VEL, 0, 0),
      //     // RamseteCommand passes volts to the callback
      //     driveTrain::tankDriveVolts,
      //     driveTrain
      //   );
        
      // driveTrain.resetOdometry(exampleTrajectory.getInitialPose());

      //wpiLIB has it as a return function because they're not using a void function to call the autonomous command
      // ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0,0));
      
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