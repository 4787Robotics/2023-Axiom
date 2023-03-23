// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.MotorController;

public class AutoMovements extends CommandBase {
  /** Creates a new DriveForwardAuto. */
  DriveTrain driveTrain;
  MotorController motorController;
  double timeStarted;
  boolean isFinished;
  boolean isForward;
  boolean isSlamming;

  public AutoMovements(DriveTrain m_driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    isForward = true;
    isSlamming = false;
  }

  public AutoMovements(DriveTrain m_driveTrain, boolean m_isForward) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    isForward = m_isForward;
  }

  public AutoMovements(MotorController m_motorController) {
    motorController = m_motorController;
    isSlamming = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isSlamming) { //Slamming arm auto
      if (timeStarted == 0) {
        timeStarted = Timer.getFPGATimestamp();
      }
      motorController.ArmMove(-0.3);
      if (Timer.getFPGATimestamp() > (timeStarted + 1.2)) {
        driveTrain.driveRobot(false, 0, 0);
        this.cancel();
        isFinished = true;
      }
    }
    else if (isForward == true) { //Moving forward auto
      if (timeStarted == 0) {
        timeStarted = Timer.getFPGATimestamp();
      }
      driveTrain.driveRobot(false, 0.8, 0);
      if (Timer.getFPGATimestamp() > (timeStarted + 1)) {
        driveTrain.driveRobot(false, 0, 0);
        this.cancel();
        isFinished = true;
      }

    } else { //Moving backward auto
      if (timeStarted == 0) {
        timeStarted = Timer.getFPGATimestamp();
      }
      driveTrain.driveRobot(false, -0.75, 0);
      if (Timer.getFPGATimestamp() > (timeStarted + 0.7)) {
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
