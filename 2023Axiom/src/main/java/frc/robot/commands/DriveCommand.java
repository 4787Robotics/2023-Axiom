// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.CXbox;
import frc.robot.CJoystick;
import frc.robot.Constants;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;
import javax.swing.JList.DropLocation;

public class DriveCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;
  private final CXbox cxbox;
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_DriveTrain The subsystem used by this command.
   */
  public DriveCommand(DriveTrain m_DriveTrain, CXbox m_cxbox) {
    driveTrain = m_DriveTrain;
    cxbox = m_cxbox;

        
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_DriveTrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // driveTrain.tankDriveVolts(0.7, 0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.Compensation(true);
    driveTrain.driveRobot(false, -cxbox.getLeftStickYWithDeadzone(), cxbox.getRightStickXWithDeadzone());
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
