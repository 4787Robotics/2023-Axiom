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
import frc.robot.commands.DriveBackwards;

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
  private final static Balance m_balance = new Balance();
  public static DriveTrain m_driveTrain = new DriveTrain();
  private final LimeLight limeLight = new LimeLight(this);
  private final static CXbox m_cxbox = new CXbox();
  private final static CJoystick m_joystick = new CJoystick();
  private final static XboxController m_xboxController = new XboxController(1);
  public static MotorController m_motorController = new MotorController();
  private final static DriveCommand m_driveCommand = new DriveCommand(m_driveTrain, m_cxbox);
  private final static NavXAutonomousCommand m_NavXAutoCommand = new NavXAutonomousCommand(m_driveTrain, m_balance);
  private final AutoAlignAndPlace autoAlignAndPlace = new AutoAlignAndPlace(limeLight, m_driveTrain, m_balance, m_driveCommand, m_xboxController);
  private final static ArmCommand m_armCommand = new ArmCommand(m_motorController, m_cxbox, m_joystick);
  private final static AutoGripCommand m_autoGripCommand = new AutoGripCommand(m_motorController);
  private final static AutoArmPIDCommand m_autoArmPIDCommand = new AutoArmPIDCommand(m_motorController);
  private final static AutoArmStartCommand m_autoArmStartCommand = new AutoArmStartCommand(m_motorController);
  private final static RammseteAutonomousCommand m_rammseteAutonomousCommand = new RammseteAutonomousCommand();
  private final static AutoArmPIDCommand m_testArmPIDCommand = new AutoArmPIDCommand(m_motorController);
  private final static DriveBackwards m_driveBackwards = new DriveBackwards(m_driveTrain, m_motorController, m_autoGripCommand);
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
  
  public Command getAutoArmStartCommand() {
    return m_autoArmStartCommand;
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

  public Command getDriveBackwards() {
    return m_driveBackwards;
  }

  public Command getAutoCommand1() {
    // working on it DISREGARD IT -- DO NOT USE IT
    // need to find the right pathNumber for each rammsete command
    Command setUp = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 13);
    Command placeStartingCube;
    Command backOut = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 6);
    Command engageChargeStation = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 1);
    return new ParallelCommandGroup (new SequentialCommandGroup(), m_autoArmPIDCommand);
  }

  public Command getAutoCommand2a() {
    //Robot needs to: Back up, than lift arm, than drive forward (with arm still up), than open thingy, than drive backwards, than drop arm
    //Command raiseArm = new ParallelRaceGroup(new ChangeArmLevel(2, m_autoArmPIDCommand, m_motorController), m_autoGripCommand);
    //Command moveToScore = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 1);
    //Command scoreCone1 = new AutoGripOandCCommand(m_motorController, true, m_autoGripCommand);
    Command backOutToFaceCube = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 16);
    //Command pickUpCube = new AutoGripOandCCommand(m_motorController, false);
      //Command m_autoGripCommand2 = new AutoGripCommand(m_motorController);
    //Command goBackToScoreCube1 = new ParallelRaceGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 3), m_autoGripCommand2);
    //Command raiseArm2 = new ParallelRaceGroup(new ChangeArmLevel(2, m_autoArmPIDCommand, m_motorController), m_autoGripCommand2);
    //Command goBackToScoreCube2 = new ParallelRaceGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 4), m_autoGripCommand2);
    //Command scoreCube = new AutoGripOandCCommand(m_motorController, true, m_autoGripCommand2);
    //Command backOutToFaceCone = m_rammseteAutonomousCommand.(m_driveTrain, 5);
    //Command pickUpCone = new AutoGripOandCCommand(m_motorController, false);
      //Command m_autoGripCommand3 = new AutoGripCommand(m_motorController);
    //Command goBackToScoreCone1 = new ParallelRaceGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain,6), m_autoGripCommand3);
    //Command raiseArm3 = new ParallelRaceGroup(new ChangeArmLevel(2, m_autoArmPIDCommand, m_motorController), m_autoGripCommand3);
    //Command goBackToScoreCone2 = new ParallelRaceGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 7), m_autoGripCommand3);
    //Command scoreCone2 = new AutoGripOandCCommand(m_motorController, true, m_autoGripCommand3);
    return new ParallelCommandGroup(new SequentialCommandGroup(backOutToFaceCube), m_autoArmPIDCommand);
//    return new ParallelCommandGroup (new SequentialCommandGroup(raiseArm, moveToScore, scoreCone1,
//                                    backOutToFaceCube, pickUpCube, goBackToScoreCube1, raiseArm2, goBackToScoreCube2, scoreCube,
//                                    backOutToFaceCone, pickUpCone, goBackToScoreCone1, raiseArm3, goBackToScoreCone2, scoreCone2),
//                                    m_autoArmPIDCommand);
  }

  public Command getAutoCommand2b() {
    //Robot needs to: Back up, than lift arm, than drive forward (with arm still up), than open thingy, than drive backwards, than drop arm
    Command setUp = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 12);
    Command placeStartingCone = new SequentialCommandGroup(new ParallelRaceGroup(new ChangeArmLevel(2, m_autoArmPIDCommand, m_motorController), m_autoGripCommand), new AutoGripOandCCommand(m_motorController, true, m_autoGripCommand));
    Command backOutToFaceCube = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 5);
    Command pickUpCube;
    Command goBackToScoreCube = new ParallelCommandGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 10), m_autoGripCommand);
    Command placeCube;
    Command backOutToFaceCone = m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 3);
    Command pickUpCone;
    Command goBackToScoreCone = new ParallelCommandGroup(m_rammseteAutonomousCommand.getRammseteAutonomousCommand(m_driveTrain, 8), m_autoGripCommand);
    Command scoreCone;
    return new ParallelCommandGroup (new SequentialCommandGroup(), m_autoArmPIDCommand);
  } 

  public AutoAlignAndPlace getAutoAlignAndPlace() {
    return autoAlignAndPlace;
  }
}
