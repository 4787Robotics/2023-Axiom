// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Balance;
import edu.wpi.first.wpilibj.Timer;

public class ChargePad extends CommandBase {
  /** Creates a new ChargePad. */
  private DriveTrain driveTrain;
  private Balance balance;
  private boolean forward;
  private boolean balanceStarted = false;
  private boolean prevDrive = false;
  private double timeOver = 0;
  private int driveCounter = 0;
  int state;
  public ChargePad(DriveTrain m_driveTrain, Balance m_balance) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveTrain = m_driveTrain;
    balance = m_balance;
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0; //IN FRONT OF CHARGE PAD- FRONT FACES CHARGE PAD
    timeOver = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (state == 0 && balance.getPitch() < -10){
      state = 1; //On charge pad, moving forward
    }
    else if (state == 1 && balance.getPitch() > 3){
      state = 2; //On charge pad, already over the flat platform, now moving forwards at an incline
    }
    else if (state == 2 && balance.getPitch() < 3 && balance.getPitch() > -3) {
      if (timeOver == 0) {
        timeOver = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() > (timeOver + 1.5)) {
        state = 3; //Drove forward all the way off of charge pad, evened out
      }
    }
    else if (state == 3 && balance.getPitch() > 10) {
      state = 4; //Starting to get back on charge pad, moving backwards 
    }
    else if (state == 4 && balance.getPitch() < 10) {
      state = 5; //On charge pad, autobalance
    }

    SmartDashboard.putNumber("State", state);
    SmartDashboard.putNumber("ChargePitch", balance.getPitch());

    switch (state) {
      case 0:
        driveTrain.driveRobot(false, 0.7, 0);
        break;
      case 1:
        driveTrain.driveRobot(false, 0.85, 0);
        break;
      case 2:
        driveTrain.driveRobot(false, 0.5, 0);
        break;
      case 3:
        driveTrain.driveRobot(false, -0.7, 0);
        break;
      case 4:
        driveTrain.driveRobot(false, -0.6, 0);
        break;
      case 5:
        if (balanceStarted == false){
          forward = false;
          balanceStarted = true;
        }
        autobalance();
        break;
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

  public void autobalance() {
    if (forward) {
      if (balance.getPitch() < -8) {
        driveTrain.driveRobot(false, 0.45, 0);
        checkDirectionChange(true);
      } else if (balance.getPitch() >= -8 && balance.getPitch() <= 0) {
        driveTrain.driveRobot(false, 0, 0);
        checkDirectionChange(true);
      } else if (balance.getPitch() <= 5 && balance.getPitch() > 0) {
        driveTrain.driveRobot(false, -0.45, 0);
        checkDirectionChange(false);
      } else if (balance.getPitch() > 5) {
        driveTrain.driveRobot(false, -0.6, 0);
        checkDirectionChange(false);
      }
    }
    else {
      if (balance.getPitch() > 8) {
        driveTrain.driveRobot(false, -0.45, 0);
        checkDirectionChange(false);
      } else if (balance.getPitch() <= 8 && balance.getPitch() >= 0) {
        driveTrain.driveRobot(false, 0, 0);
        checkDirectionChange(false);
      } else if (balance.getPitch() >= -5 && balance.getPitch() < 0) {
        driveTrain.driveRobot(false, 0.45, 0);
        checkDirectionChange(true);
      } else if (balance.getPitch() < -5) {
        driveTrain.driveRobot(false, 0.6, 0);
        checkDirectionChange(true);
      }
    }
  }

  public void checkDirectionChange(boolean drivingForwards) {
    if (prevDrive == drivingForwards) {
      driveCounter++;
    }
    else {
      driveCounter = 0;
    }

    if (driveCounter == 3) {
      forward = drivingForwards;
    }
    
    prevDrive = drivingForwards;
  }
}
