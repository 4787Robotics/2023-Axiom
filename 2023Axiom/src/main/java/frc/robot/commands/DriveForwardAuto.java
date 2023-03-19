// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardAuto extends CommandBase {
  /** Creates a new DriveForwardAuto. */
  DriveTrain driveTrain;
  double timeStarted;
  boolean isFinished;
  boolean isForward;
  public DriveForwardAuto(DriveTrain m_driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    isForward = true;
  }

  public DriveForwardAuto(DriveTrain m_driveTrain, boolean m_isForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    isForward = m_isForward;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isForward == true) {
      if (timeStarted == 0) {
        timeStarted = Timer.getFPGATimestamp();
      }
      driveTrain.driveRobot(false, 0.8, 0);
      if (Timer.getFPGATimestamp() > (timeStarted + 0.5)) {
        driveTrain.driveRobot(false, 0, 0);
        this.cancel();
        isFinished = true;
      }
    } else {
      if (timeStarted == 0) {
        timeStarted = Timer.getFPGATimestamp();
      }
      driveTrain.driveRobot(false, -0.75, 0);
      if (Timer.getFPGATimestamp() > (timeStarted + 0.3)) {
        driveTrain.driveRobot(false, 0, 0);
        this.cancel();
        isFinished = true;
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
