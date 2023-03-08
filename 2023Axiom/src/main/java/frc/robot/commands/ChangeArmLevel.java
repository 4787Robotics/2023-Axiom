// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.AutoArmPIDCommand;
import java.util.Timer;
import frc.robot.subsystems.MotorController;

public class ChangeArmLevel extends CommandBase {
  /** Creates a new changeArmLevel. */

  private Timer timer = new Timer();
  private boolean finished = false;
  int levelToCheck;
  MotorController m_MotorController;

  public ChangeArmLevel(int level, AutoArmPIDCommand autoArmPIDCommand, MotorController subsystem) {
    // Use addRequirements() here to declare subsystem dependencies. 
    autoArmPIDCommand.level = level;
    levelToCheck = level;
    m_MotorController = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (levelToCheck == 0) {
      if (m_MotorController.ArmEncoder.getPosition() < Constants.LOW_LEVEL + 0.5 && m_MotorController.ArmEncoder.getPosition() > Constants.LOW_LEVEL - 0.5) {
        finished = true;
      }
    } else if (levelToCheck == 1) {
      if (m_MotorController.ArmEncoder.getPosition() < Constants.MID_LEVEL + 0.5 && m_MotorController.ArmEncoder.getPosition() > Constants.MID_LEVEL - 0.5) {
        finished = true;
      }
    } else if (levelToCheck == 2) {
      if (m_MotorController.ArmEncoder.getPosition() < Constants.HIGH_LEVEL + 0.5 && m_MotorController.ArmEncoder.getPosition() > Constants.HIGH_LEVEL - 0.5) {
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
