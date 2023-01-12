// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  NetworkTable table;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;  

  double distance = 0;

  public LimeLight() {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
  }

  public double getX() {
    return tx.getDouble(0.0);
  }

  public double getY() {
    return ty.getDouble(0.0);
  }

  public double getArea() {
    return ta.getDouble(0.0);
  }

  public double calculateDistance() {
    distance = 0;
    double area = getArea();
    if (area < 1) {
      distance = -1; //-1 means the limelight does not see a target
    } else {  //calculate the distance based on the area of the target and some constants that we found experimentally.  This is not perfect, but it works for our robot.  
      distance = (0.0028 * Math.pow(area, 2)) + (-0.5 * area) + 50;   //distance in inches from limelight to target based on ta value and a curve fit of experimental data points taken by measuring actual distances vs values of ta reported by Limelight for those distances with reflective tape as a target at different angles to camera lens axis

      SmartDashboard.putNumber("Distance", distance);

      return distance;     //returns calculated distance in inches from limelight to reflective tape target or -1 if no valid targets are seen by Limelight camera (i.e., tv=0)

       } 

       return 0;   //this should never happen because we check for tv=0 above and return -1 if true, but just in case...
  }

  public void updateDashboard() {
    SmartDashboard.putNumber("Distance", distance);
    SmartDashboard.putNumber("LimelightX", getX());
    SmartDashboard.putNumber("LimelightY", getY());
    SmartDashboard.putNumber("LimelightArea", getArea());
    /*System.out.println("LimelightX" + Double.toString(getX()));
    System.out.println("LimelightY" + Double.toString(getY()));
    System.out.println("LimelightArea" + Double.toString(getArea()));*/
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimelightX", getX());   //puts the Limelight's x-axis offset of crosshair from center of target in degrees on the SmartDashboard so we can see it on the Driver Station and tune our aiming code to keep this value as close to zero as possible when targeting reflective tape targets.  

    SmartDashboard.putNumber("LimelightY", getY());   //puts the Limelight's y-axis offset of crosshair from center of target in degrees on the SmartDashboard so we can see it on the Driver Station and tune our aiming code to keep this value as close to zero as possible when targeting reflective tape targets.  

    SmartDashboard.putNumber("LimelightArea", getArea());     // puts ta (target area) reported by Limeligt camera for current frame on Shart Dashbaord so we can use these values with actual measured distances between limeligh camera and target at different angles relative to camera lens axis, then curve fit those data points using a graphing utility like Excel or Google Sheets to come up with a formula that calculates distance from Limelight camera to target based on ta value reported by Limelight for current frame.  This is not perfect, but it works well enough for our robot.  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
