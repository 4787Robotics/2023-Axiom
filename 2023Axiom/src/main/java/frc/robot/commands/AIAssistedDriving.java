// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AIAssistedDriving extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final LimeLight limeLight;
  private long startTime;

  private double[] tagsFound;
  private final double[] initialTags = {0,0,0,0};
  private boolean isCheckingForAllAprilTags;
  private boolean isFindingClosestAprilTag;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AIAssistedDriving(LimeLight subsystem) {
    limeLight = subsystem;
    isCheckingForAllAprilTags = false;
    isFindingClosestAprilTag = false;
    tagsFound = initialTags;
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

  public double findClosestAprilTagId() {
    isFindingClosestAprilTag = true;
    limeLight.setPipeline(Constants.ALL_APRILTAG_IDS_PIPELINE);
    restartTimer();
    while (getElapsedSeconds() < 0.01) {
      isFindingClosestAprilTag = false;
      if (limeLight.getTagID() != 0) {
        return limeLight.getTagID();
      }
    }

    isFindingClosestAprilTag = false;
    return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double closestId = findClosestAprilTagId();

    while (isFindingClosestAprilTag) {}

    /*
    - decide which target to choose
    - find distance to apriltag
    - turn parallel to target
    - move in x direction towards parallel of the target
    - turn 90 degrees such that the robot is facing the target
    - move set distance to place at certain height
    - place game piece
    - change to teleop
     */


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

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
