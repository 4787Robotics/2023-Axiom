// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.TurnAngle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class AutoAlignAndPlace extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Balance balance;
  private final DriveTrain driveTrain;
  private final LimeLight limeLight;
  private final double fieldOfViewX = 63.3;
  private final double fieldOfViewY = 49.7;
  private long startTime;

  private double[] tagsFound;
  private final double[] initialTags = {0,0,0,0};
  private boolean isCheckingForAllAprilTags;
  private volatile boolean isFindingClosestAprilTag;
  private Command teleopCommand;

  /**
   * Creates a new ExampleCommand.
   *
   * @param LL The subsystem used by this command.
   */
  public AutoAlignAndPlace(LimeLight m_limeLight, DriveTrain m_driveTrain, Balance m_balance, Command m_teleopCommand) {
    teleopCommand = m_teleopCommand;
    driveTrain = m_driveTrain;
    balance = m_balance;
    limeLight = m_limeLight;
    isCheckingForAllAprilTags = false;
    isFindingClosestAprilTag = false;
    tagsFound = initialTags;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_limeLight);
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
    if (!isCheckingForAllAprilTags) {
      isFindingClosestAprilTag = true;
      limeLight.setPipeline(Constants.ALL_APRILTAG_IDS_PIPELINE);
      restartTimer();
      while (getElapsedSeconds() < 0.01) {
        isFindingClosestAprilTag = false;
        if (limeLight.getTagID() != 0) {
          return limeLight.getTagID();
        }
      }
    }
    isFindingClosestAprilTag = false;
    return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double closestId = findClosestAprilTagId(); //find closest tag
    double distanceToTag = 0;
    double distanceTravelled = 0;
    double heldAngle = 0;
    double heldTurnAngle = 0;
    double distanceToParallelTag = 0;
    double distanceToPerpendicularTag = 0;
    TurnAngle turnAngle;
    MoveTo moveTo;

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

    //wait for execution to return a value
    while (isFindingClosestAprilTag) {
      Thread.onSpinWait();
    }

    //calculate distance only if grid tag is found
    if (closestId != 4.0) {
      distanceToTag = limeLight.calculateDistance(Constants.LIMELIGHT_APRILTAG_GRID_HEIGHT);
    } else if (closestId == 0) {
      cancel();
      System.out.println("No AprilTag Found");
    }
    SmartDashboard.putNumber("Distance", distanceToTag);

    if (limeLight.getXAngle() > 0) {
      heldTurnAngle =  balance.getHeading() - 90;
      heldAngle = (balance.getHeading() + limeLight.getXAngle()) - 90;
      turnAngle = new TurnAngle(driveTrain, balance, -heldTurnAngle);
      turnAngle.schedule();
    } else {
      heldTurnAngle =  (360 - balance.getHeading()) - 90;
      heldAngle = ((360 - balance.getHeading()) + limeLight.getXAngle()) - 90;

      turnAngle = new TurnAngle(driveTrain, balance, heldTurnAngle);
      turnAngle.schedule();
    }

    distanceToPerpendicularTag = Math.sin(Math.toRadians(heldAngle)) * distanceToTag;
    distanceToParallelTag = Math.cos(Math.toRadians(heldAngle)) * distanceToTag;

    while (!turnAngle.isFinished()) {
      Thread.onSpinWait();
    }

    turnAngle.cancel();
    System.out.println("distanceToTag" + distanceToTag);
    System.out.println("distanceToPerpendicularTag" + distanceToPerpendicularTag);
    System.out.println("distanceToParallelTag" + distanceToParallelTag);

    cancel();

    //drive distanceToParallelTag
    /*moveTo = new MoveTo(driveTrain, balance, distanceToParallelTag);
    moveTo.schedule();
    while (!moveTo.isFinished()) {
      Thread.onSpinWait();
    }*/
    //turn towards target

    //forward/backward adjust
    //arm command
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isCheckingForAllAprilTags = false;
    isFindingClosestAprilTag = false;
    tagsFound = initialTags;
    if (teleopCommand != null) {
      teleopCommand.cancel();
    }
    assert teleopCommand != null;
    teleopCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
