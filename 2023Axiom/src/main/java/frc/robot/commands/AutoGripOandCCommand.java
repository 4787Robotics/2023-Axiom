// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorController;
import frc.robot.commands.AutoGripOandCCommand;
import edu.wpi.first.wpilibj.Timer;

public class AutoGripOandCCommand extends CommandBase {
  /** Creates a new AutoArmCommnd. */
  private final MotorController m_subsystem; 
  private final AutoGripCommand m_AutoGripCommand;
  public boolean AutoGrabbing;
  public boolean finished = false;
  public AutoGripOandCCommand(MotorController subsystem, AutoGripCommand AutoGripCommand) {
    m_subsystem = subsystem;
    m_AutoGripCommand = AutoGripCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (AutoGrabbing){
      m_subsystem.Intake(.1);
    } else if (!AutoGrabbing){
      m_subsystem.Intake(-.1);
    }
    Timer.delay(1);
    m_subsystem.Intake(0);
    finished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
