// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Balance;
import edu.wpi.first.math.MathUtil;

public class TurnAngle extends CommandBase {
  /** Creates a new TurnAngle. */
  private DriveTrain driveTrain;
  private Balance balance;
  private double turnTo;
  private double headingTo;

  public TurnAngle(DriveTrain m_driveTrain, Balance m_balance, double TurnTo) {
    driveTrain = m_driveTrain;
    balance = m_balance;
    turnTo = TurnTo;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain, m_balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    headingTo = balance.getLinearHeading() + turnTo;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (balance.getLinearHeading() < headingTo + 1 && balance.getLinearHeading() > headingTo - 1) {
      driveTrain.driveRobot(false, 0, 0);
      this.cancel();
    }
    else {
      double speed = MathUtil.clamp(balance.calculatePID(balance.getLinearHeading(), headingTo), -0.5, 0.5);
      if (speed < 0) {
        speed = MathUtil.clamp(speed, -0.5, -0.35);
      }
      else if (speed > 0) {
        speed = MathUtil.clamp(speed, 0.35, 0.5);
      }
      System.out.println("speed: " + speed);
      driveTrain.driveRobot(false, 0, speed);
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
