// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants;
import frc.robot.subsystems.Balance;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.RobotController;

public class MoveTo extends CommandBase {
  /** Creates a new TurnAngle. */
  private final DriveTrain driveTrain;
  private Balance balance;
  private double meters; // positive meters = forward while negative meters = backward
  private double headingTo;
  private double millisecondsToRun; // This should run 1000ms = 1 s.
  private double initTime = RobotController.getFPGATime();

  public MoveTo(DriveTrain m_driveTrain, Balance m_balance, double meters) {
    driveTrain = m_driveTrain;
    balance = m_balance;
        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain, m_balance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    millisecondsToRun = (meters/Constants.KV_VOLT_SECONDS_PER_METER)/1000;
    while(RobotController.getFPGATime() - initTime <= millisecondsToRun) {
        if (meters > 0) {
            driveTrain.driveRobot(false, 0.1, 0);
        }
        else if (meters < 0) {
            driveTrain.driveRobot(false, 0.1, 0);
        }
        else {
            driveTrain.driveRobot(false, 0, 0);
        }
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
