// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
  double distance = 0;

  //Initialize fields and get NetworkTable for the first limelight
  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    tid = table.getEntry("tid");
    getpipe = table.getEntry("getpipe");
    pipeline = table.getEntry("pipeline");
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

  public double calculateDistance() {

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
    
    //using known variables (angles, heights, etc.)
    double area = getArea();
    if (area == 0) {
      distance = -1; //-1 means the limelight does not see a target
    } else {  //calculate the distance based on the area of the target and some constants that we found experimentally.  This is not perfect, but it works for our robot. 
      double angleToGoalDegrees = Constants.LimelightMountAngle + getYAngle();
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); 

      distance = (Constants.GoalHeight - Constants.LimelightLensHeight)/Math.tan(angleToGoalRadians);   
    }

    return distance; //returns calculated distance in inches from limelight to reflective tape target or -1 if no valid targets are seen by Limelight camera (i.e., tv=0)
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Distance", calculateDistance());
    SmartDashboard.putNumber("LimelightX", getXAngle());
    SmartDashboard.putNumber("LimelightY", getYAngle());
    SmartDashboard.putNumber("LimelightArea", getArea());
    SmartDashboard.putNumber("CurrentTargetedTagID", getTagID());
    SmartDashboard.putNumber("CurrentPipline", getPipline());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
