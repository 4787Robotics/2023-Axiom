package frc.robot.commands;

import frc.robot.subsystems.Balance;
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
}
