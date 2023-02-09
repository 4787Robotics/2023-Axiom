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

  public TurnAngle(DriveTrain m_driveTrain, Balance m_balance, double TurnTo) {
    driveTrain = m_driveTrain;
    balance = m_balance;
    turnTo = TurnTo;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain, m_balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.driveRobot(false, 0, MathUtil.clamp(balance.calculatePID(balance.getHeading(), balance.getHeading() + turnTo), -0.7, 0.7));
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
