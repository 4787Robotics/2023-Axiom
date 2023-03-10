// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Balance;
import frc.robot.subsystems.Scorekeeper;
import frc.robot.subsystems.ScoringArea;
//import frc.robot.subsystems.Scorekeeper;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.subsystems.ScoringArea;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.CXbox;
import frc.robot.commands.RammseteAutonomousCommand;
import frc.robot.commands.TestTurnAngle;
import frc.robot.commands.TurnAngle;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private XboxController m_controller;
  private Command m_autonomousCommand;
  private Command m_driveCommand;
  private Command m_autoAlignAndPlaceCommand;
  private Command m_autoArmPIDCommand;
  private Command m_autoGripCommand;
  private Command m_autoGripOandCCommand;
  private Command m_autoArmStartCommand;
  private Command m_cancelPlaceCommand;
  private Command m_armCommand;
  private Command m_teleopCommand;
  private Command m_armPIDCommand;
  private Command m_pathCommand;
  private RobotContainer m_robotContainer;
  private boolean debounce = true;

  String trajectoryJSON_1 = "paths/preloadedCone2a.wpilib.json";
  String trajectoryJSON_2 = "paths/getCube2a.wpilib.json";
  String trajectoryJSON_3 = "paths/placeCube2aPart1.wpilib.json";
  String trajectoryJSON_4 = "paths/placeCube2aPart2.wpilib.json";
  String trajectoryJSON_5 = "paths/getCone2a.wpilib.json";
  String trajectoryJSON_6 = "paths/placeCone2aPart1.wpilib.json";
  String trajectoryJSON_7 = "paths/placeCone2aPart2.wpilib.json";

  String trajectoryJSON_8 = "paths/preloadedCone2b.wpilib.json";
  String trajectoryJSON_9 = "paths/getCube2b.wpilib.json";
  String trajectoryJSON_10 = "paths/placeCube2bPart1.wpilib.json";
  String trajectoryJSON_11 = "paths/placeCube2bPart2.wpilib.json";
  String trajectoryJSON_12 = "paths/getCone2b.wpilib.json";
  String trajectoryJSON_13 = "paths/placeCone2bPart1.wpilib.json";
  String trajectoryJSON_14 = "paths/placeCone2bPart2.wpilib.json";

  String trajectoryJSON_15 = "paths/preloadedCubev1.wpilib.json";
  String trajectoryJSON_16 = "paths/moveBackv1.wpilib.json";
  String trajectoryJSON_17 = "paths/chargeStation.wpilib.json";

  
  public static Trajectory[] trajectoryArray = new Trajectory[17];

  int i = 0;
  public void readTrajectory(String trajectoryJSON){
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      Trajectory trajectory = new Trajectory();
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      trajectoryArray[i] = trajectory;
      i++;
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
  }

  //private CXbox cxbox = new CXbox();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_controller = new XboxController(1);
    m_robotContainer = new RobotContainer();
    m_driveCommand = m_robotContainer.getDriveCommand();
    //m_autonomousCommand = m_robotContainer.getNavXAutoCommand();
    m_autoAlignAndPlaceCommand = m_robotContainer.getAutoAlignAndPlace();
    m_autonomousCommand = m_robotContainer.getNavXAutoCommand();
    m_armCommand = m_robotContainer.getArmCommand();
    m_teleopCommand = new ParallelCommandGroup(m_driveCommand, m_armCommand);
    m_autoArmPIDCommand = m_robotContainer.getAutoArmPIDCommand();
    m_autoGripCommand = m_robotContainer.getAutoGripCommand();
    m_autoArmStartCommand = m_robotContainer.getAutoArmStartCommand();
    //m_autoAlignAndPlaceCommand = m_robotContainer.getAutoAlignAndPlace();
    readTrajectory(trajectoryJSON_1);
    readTrajectory(trajectoryJSON_2);
    readTrajectory(trajectoryJSON_3);
    readTrajectory(trajectoryJSON_4);
    readTrajectory(trajectoryJSON_5);
    readTrajectory(trajectoryJSON_6);
    readTrajectory(trajectoryJSON_7);
    readTrajectory(trajectoryJSON_8);
    readTrajectory(trajectoryJSON_9);
    readTrajectory(trajectoryJSON_10);
    readTrajectory(trajectoryJSON_11);
    readTrajectory(trajectoryJSON_12);
    readTrajectory(trajectoryJSON_13);
    readTrajectory(trajectoryJSON_14);
    readTrajectory(trajectoryJSON_15);
    readTrajectory(trajectoryJSON_16);
    readTrajectory(trajectoryJSON_17);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  
  @Override
  
  public void autonomousInit() {
   
    //m_autonomousCommand = m_robotContainer.getAutoCommand2a();
    // m_autoArmStartCommand.schedule();


    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      // m_pathCommand.cancel();
    }

    // assert m_autonomousCommand != null;
    // m_autonomousCommand.schedule();
    // m_pathCommand.schedule();
    //TestTurnAngle m_testTurnAngleCommand = new TestTurnAngle(m_robotContainer.getBalance(), m_robotContainer.getDriveTrain(), 90);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_teleopCommand != null) {
      m_teleopCommand.cancel();
    }

    m_teleopCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if (m_controller.getStartButtonPressed()) {
      if (debounce) {
        debounce = false;
        if (m_autoAlignAndPlaceCommand.isScheduled()) {
          m_autoAlignAndPlaceCommand.cancel();
        }
        m_autoAlignAndPlaceCommand.schedule();
        System.out.println("teleopcancel");
        if (m_teleopCommand != null) {
          m_teleopCommand.cancel();
        }
      }
    } else if (m_controller.getStartButtonReleased()) {
      debounce = true;
    }

    if (!m_autoAlignAndPlaceCommand.isScheduled()) {
      m_teleopCommand.schedule();
    }
      
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    /*
    CXbox.getXboxDpad();
    CXbox.XboxADown();
    CXbox.XboxBDown();
    CXbox.XboxXDown();
    CXbox.XboxYDown();
    CXbox.XboxLStickDown();
    CXbox.XboxRStickDown();
    CXbox.XboxLBumperDown(); 
    CXbox.XboxRBumperDown();
    CXbox.getLeftTriggerWithDeadzone();
    CXbox.getRightTriggerWithDeadzone();
    CXbox.getLeftStickXWithDeadzone();
    CXbox.getLeftStickYWithDeadzone();
    CXbox.getRightStickXWithDeadzone();
    CXbox.getRightStickYWithDeadzone();
    //*/

    /*
    CJoystick.getJoystickPOV();
    CJoystick.getJoystickXWithDeadzone();
    CJoystick.getJoystickYWithDeadzone();
    CJoystick.getJoystickRotationWithDeadzone();
    CJoystick.getJoystickThrottle();
    CJoystick.joystickButton1Down();
    CJoystick.joystickButton2Down();
    CJoystick.joystickButton3Down();
    CJoystick.joystickButton4Down();
    CJoystick.joystickButton5Down();
    CJoystick.joystickButton6Down();
    CJoystick.joystickButton7Down();
    CJoystick.joystickButton8Down();
    CJoystick.joystickButton9Down();
    CJoystick.joystickButton10Down();
    CJoystick.joystickButton11Down();
    CJoystick.joystickButton12Down();
    //*/
    Scorekeeper.updateDashboard();
  }
}
