// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.CXbox;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlignAndPlace;

public class LimeLight extends SubsystemBase {
  //NetworkTable fields
  NetworkTable table;
  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;  
  NetworkTableEntry tid;
  NetworkTableEntry getpipe;
  NetworkTableEntry pipeline;

  //Other fields
  RobotContainer m_robotContainer;
  XboxController m_controller;
  AutoAlignAndPlace m_autoPlaceCommand;
  Command m_teleopCommand;

  //Initialize fields and get NetworkTable for the first limelight
  public LimeLight(RobotContainer robotContainer) {
    m_robotContainer = robotContainer;
    m_controller = new XboxController(Constants.XCONTROLLER_PORT);
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    getpipe = table.getEntry("getpipe");
    pipeline = table.getEntry("pipeline");
  }

  public void setTeleopCommand(Command teleopCommand) {
    m_teleopCommand = teleopCommand;
  }

  //Returns the horizontal angle the detected object 
  public double getXAngle() {
    return tx.getDouble(0.0);
  }

  public double getYAngle() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public double getTagID() {
    return tid.getDouble(0.0);
  }

  public double getPipline() {
    return getpipe.getDouble(0.0);
  }

  public boolean hasTarget() {
    if (tv.getDouble(0.0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /** 
   * Set pipline number between 0-9.
  */
  public void setPipeline(double pipelineNumber) {
    pipeline.setValue(pipelineNumber);
  }

  public void setTargetedAprilTagId(double targetedAprilTagId) {
    setPipeline(targetedAprilTagId + 1);
  }

  public double calculateDistance(double targetHeight) {

    double distance = 0;

    //with area (doesnt fucking work im ass mb)
    /*double area = getArea();
    if (area < 1) {
      distance = -1; //-1 means the limelight does not see a target
    } else {  //calculate the distance based on the area of the target and some constants that we found experimentally.  This is not perfect, but it works for our robot.  
      distance = (0.0028 * Math.pow(area, 2)) + (-0.5 * area) + 50;   //distance in inches from limelight to target based on ta value and a curve fit of experimental data points taken by measuring actual distances vs values of ta reported by Limelight for those distances with reflective tape as a target at different angles to camera lens axis

      SmartDashboard.putNumber("Distance", distance);

      return distance;     //returns calculated distance in inches from limelight to reflective tape target or -1 if no valid targets are seen by Limelight camera (i.e., tv=0)

       } 

       return 0;*/
    
    //Using known variables (angles, heights, etc.)
    //From limelight documentation
    /*if (hasTarget()) {
      distance = -1; //-1 means the limelight does not see a target
    } else {  //calculate the distance based on the area of the target and some constants that we found experimentally.  This is not perfect, but it works for our robot.
      double angleToGoalDegrees = Constants.LIMELIGHT_MOUNT_ANGLE + getYAngle();
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

      distance = (Constants.LIMELIGHT_GOAL_HEIGHT - Constants.LIMELIGHT_LENS_HEIGHT)/Math.tan(angleToGoalRadians);
    }*/

    //Distance calculation from https://www.chiefdelphi.com/t/calculating-distance-to-vision-target/387183/6
    if (hasTarget()) {
      distance = 0; //-1 means the limelight does not see a target
    } else {
      double z = 1 / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));
      double y = getYAngle() / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));
      double x = getXAngle() / (Math.sqrt(1 + Math.pow(Math.tan(getYAngle()), 2) + Math.pow(Math.tan(getXAngle()), 2)));

      double scaleFactor = (targetHeight - Constants.LIMELIGHT_LENS_HEIGHT) / y;

      distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2)) * scaleFactor;
    }

    //20.32 accounts for the distance from the limelight to the center of the robot
    return distance + 20.32; //returns calculated distance in inches from limelight to reflective tape target or 0 if no valid targets are seen by Limelight camera (i.e., tv=0)
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("LimelightX", getXAngle());
    SmartDashboard.putNumber("LimelightY", getYAngle());
    SmartDashboard.putNumber("LimelightArea", getArea());
    SmartDashboard.putNumber("CurrentTargetedTagID", getTagID());
    SmartDashboard.putNumber("CurrentPipline", getPipline());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler
    m_autoPlaceCommand = m_robotContainer.getAutoAlignAndPlace();
    if (m_controller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
      if (m_autoPlaceCommand.isScheduled()) {
          m_autoPlaceCommand.cancel();
          System.out.println("finished");
          //m_teleopCommand.schedule();
      }
    } 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    if (m_controller.getLeftX() > Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftX() < -Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftY() > Constants.LEFT_TRIGGER_DEAD_ZONE || m_controller.getLeftY() < -Constants.LEFT_TRIGGER_DEAD_ZONE) {
      if (m_autoPlaceCommand.isScheduled()) {
          m_autoPlaceCommand.cancel();
          System.out.println("finished");
      }
    }
  }
}
