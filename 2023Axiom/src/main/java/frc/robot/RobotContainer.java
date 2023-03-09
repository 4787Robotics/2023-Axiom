// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.PrinterURI;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.AutoAlignAndPlace;
import frc.robot.commands.AutoArmPIDCommand;
import frc.robot.commands.AutoGripCommand;
import frc.robot.commands.AutoGripOandCCommand;
import frc.robot.commands.ChangeArmLevel;
import frc.robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RammseteAutonomousCommand;
// import frc.robot.commands.AutonomousCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.NavXAutonomousCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.MotorController;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.AutoArmPIDCommand;
import frc.robot.commands.AutoGripCommand;
import frc.robot.commands.AutoArmStartCommand;
import frc.robot.subsystems.Balance;
import frc.robot.commands.ChangeArmLevel;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final LimeLight limeLight = new LimeLight(this);
  private final static CXbox m_cxbox = new CXbox();
  private final static CJoystick m_joystick = new CJoystick();
  private final static Balance m_balance = new Balance();
  private final static XboxController m_xboxController = new XboxController(1);
  public static DriveTrain m_driveTrain = new DriveTrain();
  public static MotorController m_motorController = new MotorController();
  private final static DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, m_cxbox);
  private final static NavXAutonomousCommand m_NavXAutoCommand = new NavXAutonomousCommand(m_driveTrain, m_balance);
  private final AutoAlignAndPlace autoAlignAndPlace = new AutoAlignAndPlace(limeLight, m_driveTrain, m_balance, m_driveCommand, m_xboxController);
  private final static ArmCommand m_armCommand = new ArmCommand(m_motorController, m_cxbox, m_joystick);
  private final static AutoGripCommand m_autoGripCommand = new AutoGripCommand(m_motorController);
  private final static AutoArmPIDCommand m_autoArmPIDCommand = new AutoArmPIDCommand(m_motorController);
  private final static AutoArmStartCommand m_autoArmStartCommand = new AutoArmStartCommand(m_motorController);
  private final static AutoGripOandCCommand m_autoGripOandCCommand = new AutoGripOandCCommand(m_motorController, false, m_autoGripCommand);
  private final static RammseteAutonomousCommand m_rammseteAutonomousCommand = new RammseteAutonomousCommand();
  private final static AutoArmPIDCommand m_testArmPIDCommand = new AutoArmPIDCommand(m_motorController);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  /*
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public CXbox getCXbox() {
    return m_cxbox;
  }

  public Command getNavXAutoCommand() {
    // I'LL FIGURE THIS OUT AT SOME POINT
    return m_NavXAutoCommand;
  }

  public Command getDriveCommand() {
    return m_driveCommand;
  }

  public Command getArmCommand() {
    return m_armCommand;
  }

  public Command getAutoArmPIDCommand() {
    return m_autoArmPIDCommand;
  }

  public Command getAutoGripCommand() {
    return m_autoGripCommand;
  }
  
  public Command getAutoGripOandCCommand() {
    return m_autoGripOandCCommand;
  }
  
  public Command getAutoArmStartCommand() {
    return m_autoGripCommand;
  }

  public MotorController getMotorController() {
    return m_motorController;
  }

  public Balance getBalance() {
    return m_balance;
  }

  public DriveTrain getDriveTrain() {
    return m_driveTrain;
  }

  public Command getArmPIDCommand() {
    return m_testArmPIDCommand;
  }

  public Command getAutoCommand1() {
    // working on it DISREGARD IT -- DO NOT USE IT
    // need to find the right pathNumber for each rammsete command
    Command placeStartingCube;
    Command backOut = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 0);
    Command engageChargeStation = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 0);
    return new ParallelCommandGroup (new SequentialCommandGroup(), m_autoArmPIDCommand);
  }

  public Command getAutoCommand2a() {
    //Robot needs to: Back up, than lift arm, than drive forward (with arm still up), than open thingy, than drive backwards, than drop arm
    Command placeStartingCone = new SequentialCommandGroup(new ParallelRaceGroup(new ChangeArmLevel(2, m_autoArmPIDCommand, m_motorController), m_autoGripCommand), new AutoGripOandCCommand(m_motorController, true, m_autoGripCommand));
    Command backOutToFaceCube = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 1);
    Command pickUpCube;
    Command goBackToScoreCube = new ParallelCommandGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 2), m_autoGripCommand);
    Command placeCube;
    Command backOutToFaceCone = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 3);
    Command pickUpCone;
    Command goBackToScoreCone = new ParallelCommandGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain,43), m_autoGripCommand);
    Command scoreCone;
    return new ParallelCommandGroup (new SequentialCommandGroup(), m_autoArmPIDCommand);
  }

  public AutoAlignAndPlace getAutoAlignAndPlace() {
    return autoAlignAndPlace;
  }
}
