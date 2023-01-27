// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Balance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_teleopCommand = m_robotContainer.getTeleopCommand();

    Shuffleboard.getTab("New Tab").add(m_robotContainer.getBalance().getGyro());
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
  /*
  @Override
  
  public void autonomousInit() {
    /*
    System.out.println("AUTO INIT");
    m_robotContainer.getBalance().setHeadingAdjust();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    
  } */
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
    m_teleopCommand = RobotContainer.getTeleopCommand();
    if (m_teleopCommand != null) {
      m_teleopCommand.cancel();
    }
    m_teleopCommand.schedule();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

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
    */

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
    */
  }
}
