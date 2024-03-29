// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorController;
import frc.robot.commands.AutoGripOandCCommand;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoGripOandCCommand extends CommandBase {
  /** Creates a new AutoArmCommnd. */
  private final MotorController m_subsystem; 
  public boolean opening;
  public boolean finished = false;
  private Command autoGripCommand;
  private double startTime = 0;
  public AutoGripOandCCommand(MotorController subsystem, boolean isOpening, Command m_autoGripCommand) {
    m_subsystem = subsystem;
    opening = isOpening;
    autoGripCommand = m_autoGripCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }


  public AutoGripOandCCommand(MotorController subsystem, boolean isOpening) {
    m_subsystem = subsystem;
    opening = isOpening;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (autoGripCommand != null) {
      autoGripCommand.cancel();
    }
    finished = false;
    startTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // - is opening, + is closing
    if (startTime == 0) {
      startTime = Timer.getFPGATimestamp();
    }
    else if (Timer.getFPGATimestamp() < startTime + 0.3) {
      m_subsystem.Intake(-0.3); //opening. MUST BE POSITIVE
    }
    else if (Timer.getFPGATimestamp() < startTime + 1) {
      if (opening){
        m_subsystem.Intake(0.3); //opening. MUST BE POSITIVE
      } else if (!opening){
        m_subsystem.Intake(-1); //closing. MUST BE NEGATIVE
      }
    } else {
      finished = true;
      this.cancel();
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
