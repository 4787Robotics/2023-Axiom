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

    public TestTurnAngle(DriveTrain m_driveTrain, RamseteCommand m_ramseteCommand, TrajectoryConfig m_config) {
        driveTrain = m_driveTrain;
        addRequirements(driveTrain);

        trajectory = TrajectoryGenerator.generateTrajectory(
                //start
                new Pose2d(0,0,new Rotation2d(0)),
                //turn
                List.of(new Translation2d(0,0)),
                //end
                new Pose2d(0, 0, new Rotation2d(90)),
                // Pass config
                m_config);

        ramseteCommand = m_ramseteCommand;
    }

    @Override
    public void initialize() {
        driveTrain.resetOdometry(trajectory.getInitialPose());
    }

    @Override
    public void execute() {
        ramseteCommand.execute();
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.tankDriveVolts(0, 0);
    }

    @Override
    public boolean isFinished() {return false;}
}