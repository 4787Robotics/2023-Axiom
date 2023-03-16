package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

public class ChangeTurnAngleAndDistance extends CommandBase {
    private double heldAngle;
    private double heldParallelDistance;
    private double heldPerpendicularDistance;

    public ChangeTurnAngleAndDistance() {
        heldAngle = 0;
        heldParallelDistance = 0;
        heldPerpendicularDistance = 0;
    }

    public void setHeldAngle(double angle) {
        heldAngle = angle;
    }

    public void setHeldParallelDistance(double distance) {
        heldParallelDistance = distance;
    }

    public void setHeldPerpendicularDistance(double distance) {
        heldPerpendicularDistance = distance;
    }

    public double getHeldAngle() {
        return heldAngle;
    }

    public double getHeldParallelDistance() {
        return heldParallelDistance;
    }

    public double getHeldPerpendicularDistance() {
        return heldPerpendicularDistance;
    }
}