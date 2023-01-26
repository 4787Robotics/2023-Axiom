// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final static DriveTrain m_subsystem = new DriveTrain();
  static XboxController inputController = new XboxController(0);
  private final static DriveCommand m_teleopCommand = new DriveCommand(m_subsystem, inputController);
  
  // The robot's subsystems and commands are defined here...
  
  // private final DriveTrain m_exampleSubsystem = new DriveTrain();
  // private final XboxController inputController = new XboxController(0);

  // private final DriveCommand m_teleopCommand = new DriveCommand(m_exampleSubsystem, inputController);

  private final Balance m_balance = new Balance();

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

   /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    //I'LL FIGURE THIS OUT AT SOME POINT
    return m_teleopCommand;
  }

  */
  public static Command getTeleopCommand(){
   
    return m_teleopCommand;
  }

  public Balance getBalance() {
    return m_balance;
  }
}
