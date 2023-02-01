// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AIAssistedDriving extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimeLight limeLight;
  private long startTime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AIAssistedDriving(LimeLight subsystem) {
    limeLight = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  private void restartTimer() {
    startTime = System.currentTimeMillis();
  }

  private double getElapsedSeconds() {
    long elapsedTime = System.currentTimeMillis() - startTime;
    long elapsedSeconds = elapsedTime / 1000;
    return (double) elapsedSeconds;
  }

  public boolean checkForAprilTag(int iD) {
    limeLight.setTargetedAprilTagId(iD);
    return limeLight.hasTarget();
  }

  public double[] checkForAllAprilTags() {
    double tags[] = {0,0,0,0};
    for (int i = 0; i < 4; i++) {
      restartTimer();
      while (getElapsedSeconds() < 0.01) {
        if (checkForAprilTag(i + 1)) {
          tags[i] = 1;
          break;
        }
      }
    }
    
    return tags;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //SmartDashboard.putNumberArray("tags found", checkForAllAprilTags());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Dashboard shit
    limeLight.updateDashboard();
    SmartDashboard.putNumberArray("tags found", checkForAllAprilTags());
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
