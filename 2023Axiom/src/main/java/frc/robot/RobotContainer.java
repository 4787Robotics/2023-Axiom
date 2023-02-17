// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoAlignAndPlace;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.RammseteAutonomousCommand;
// import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.NavXAutonomousCommand;
import frc.robot.subsystems.DriveTrain;

import frc.robot.subsystems.Balance;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final LimeLight limeLight = new LimeLight();

  private final static Balance m_balance = new Balance();
  public static DriveTrain m_driveTrain = new DriveTrain();
  private final static DriveCommand m_teleopCommand = new DriveCommand(m_driveTrain);
  private final static NavXAutonomousCommand m_NavXAutoCommand = new NavXAutonomousCommand(m_driveTrain, m_balance);
  private final static RammseteAutonomousCommand m_pathCommand = new RammseteAutonomousCommand(m_driveTrain);
  private final AutoAlignAndPlace autoAlignAndPlace = new AutoAlignAndPlace(limeLight, m_driveTrain, m_balance);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /* The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getNavXAutoCommand() {
    //I'LL FIGURE THIS OUT AT SOME POINT
    return m_NavXAutoCommand;
  }

  public Command getPathCommand() {
    //I'LL FIGURE THIS OUT AT SOME POINT
    return m_pathCommand;
  }

  public Command getTeleopCommand(){
    return m_teleopCommand;
  }

  public Balance getBalance() {
    return m_balance;
  }
  
  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  public AutoAlignAndPlace getAutoAlignAndPlace() { return autoAlignAndPlace;}
}