package frc.robot.commands;

/*import frc.robot.subsystems.Balance;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;

public class TestTurnAngle extends PIDCommand {
    private Balance balance;
    private DriveTrain driveTrain;
    private PIDController pid;

    public TestTurnAngle(Balance m_balance, DriveTrain m_driveTrain, double TurnTo) {
        super(
                // The controller that the command will use
                m_balance.getPID(),
                // This should return the measurement
                () -> -m_balance.getTestHeading(),
                // This should return the setpoint (can also be a constant)
                TurnTo,
                // This uses the output
                output -> {
                    m_driveTrain.driveRobot(false, 0, output);
                },
                // Requires the drive
                m_driveTrain);
        driveTrain = m_driveTrain;
        balance = m_balance;
        pid = m_balance.getPID();
        pid.enableContinuousInput(-180, 180);
        pid.setTolerance(5, 10);
    }

    @Override
    public boolean isFinished() {
        return balance.getPID().atSetpoint();
    }
}*/

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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class TestTurnAngle extends CommandBase {
    private DriveTrain driveTrain;
    private Trajectory trajectory;
    private RamseteCommand ramseteCommand;

    public TestTurnAngle() {addRequirements(RobotContainer.m_driveTrain);}

    public Command changeRamseteCommand(DriveTrain m_driveTrain, double TurnTo) {
        System.out.println("changingturn");

        driveTrain = m_driveTrain;
        addRequirements(driveTrain);

        DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(Constants.KS_VOLTS,
                        Constants.KV_VOLT_SECONDS_PER_METER,
                        Constants.KA_VOLT_SECONDS_SQUARED_PER_METER),
                Constants.K_DRIVE_KINEMATICS,
                4
        );

        TrajectoryConfig m_config = new TrajectoryConfig(Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED, Constants.K_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
                .setKinematics(Constants.K_DRIVE_KINEMATICS) //ensures max speed is actually obeyed
                .addConstraint(autoVoltageConstraint)
                .setReversed(false); //voltage constraint

        trajectory = TrajectoryGenerator.generateTrajectory(
                List.of(new Pose2d(0, 0, new Rotation2d(0)), 
                        new Pose2d(0.001, 0, new Rotation2d(TurnTo))),
                m_config
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

        return ramseteCommand;
        //return ramseteCommand.andThen(() -> driveTrain.driveRobot(false, 0, 0));
    }

    /**
     * Returns the RamseteCommand
     * @return RamseteCommand
     */

    @Override
    public void initialize() {}

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {return false;}
}