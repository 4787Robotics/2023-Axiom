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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.MotorSafety;
import frc.robot.CXbox;
import frc.robot.CJoystick;
import frc.robot.Constants;

import java.util.concurrent.TimeUnit;

import javax.lang.model.util.ElementScanner6;

// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
//import com.revrobotics.CANSparkMax;

public class DriveCommand extends CommandBase {
  static CXbox Xbox = new CXbox();
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain driveTrain;
  // XboxController controller = new XboxController(0);
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveTrain m_DriveTrain) {
    driveTrain = m_DriveTrain;
        
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
    driveTrain.drive.arcadeDrive(Xbox.getLeftStickYWithDeadzone(), Xbox.getRightStickXWithDeadzone());
    // if(controller.getLeftY()>0.7f || controller.getLeftY()<-0.7f || controller.getRightX() > 0.7f || controller.getRightX() < -0.7f){
    //   driveTrain.drive.arcadeDrive(controller.getLeftY(), controller.getRightX());
    // }
    // else {
    //   driveTrain.drive.arcadeDrive(0, 0);
    // }                                                                                                                       
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
