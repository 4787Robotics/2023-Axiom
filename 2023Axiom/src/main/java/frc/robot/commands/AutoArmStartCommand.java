// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorController;
import edu.wpi.first.wpilibj.Timer;

public class AutoArmStartCommand extends CommandBase {
  private final MotorController m_subsystem; 
  public boolean finished2= false;
  double timeStarted;
  int i = 0;
  public AutoArmStartCommand(MotorController subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeStarted = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (i < 8) {
      m_subsystem.ArmHolderStart(.2);
      i++;
    }
    if (i == 8) {
      finished2 = true;
      this.cancel();
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished2;
  }
}
