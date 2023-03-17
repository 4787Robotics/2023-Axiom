// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MotorController;
import edu.wpi.first.wpilibj.Timer;

public class DriveBackwards extends CommandBase {
  /** Creates a new DriveBackwards. */
  DriveTrain driveTrain;
  private long startTime;
  private MotorController motorController;
  private AutoGripCommand autoGripCommand;

  public DriveBackwards(DriveTrain m_driveTrain, MotorController m_motorController, AutoGripCommand m_autoGripCommand) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    motorController = m_motorController;
    autoGripCommand = m_autoGripCommand;
  }

  private void restartTimer() {
    startTime = System.currentTimeMillis();
  }

  private double getElapsedSeconds() {
    long elapsedTime = System.currentTimeMillis() - startTime;
    long elapsedSeconds = elapsedTime / 1000;
    return (double) elapsedSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    restartTimer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (getElapsedSeconds() < 0.05) {
      driveTrain.driveRobot(false, -0.3, 0);
    } else if (getElapsedSeconds() < 0.1) {
      driveTrain.driveRobot(false, 0.4, 0);
    } else {
      AutoGripOandCCommand gCommand = new AutoGripOandCCommand(motorController, true, autoGripCommand);
      gCommand.schedule();
      this.cancel();
    }*/
    
    if (getElapsedSeconds() < 2.2) {
      driveTrain.driveRobot(false, -0.4, 0);
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
