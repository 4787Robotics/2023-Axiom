// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.CXbox;

public class NavXAutonomousCommand extends CommandBase {
  DriveTrain driveTrain;
  Balance balance;
  int i = 0;

  /** Creates a new NavXAutonomousCommand. */
  public NavXAutonomousCommand(DriveTrain m_driveTrain, Balance m_balance) {
    driveTrain = m_driveTrain;
    balance = m_balance;
    addRequirements(m_driveTrain, m_balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (i == 0) {
    //   System.out.println("NavX running");
    //   TurnAngle turnAngle = new TurnAngle(driveTrain, balance, 30);
    //   turnAngle.schedule();
    //   i++;
    // }
    driveTrain.driveRobot(true, 0.5, 0.5);
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