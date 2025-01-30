// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;

  public void LimelightSetup() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");  
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    
  }

  public double LimelightDistance(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry distanceEntry;

    //double y = ty.getDouble(0.0);
   
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);
   
    double limelightAngleDegrees = 0;

    double limelightHeightInches = 0;
    
    double goalHeightInches = 0;

    double angleToGoalDegrees = limelightAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees *(3.14159 / 160);

    double distance = (goalHeightInches - limelightHeightInches) / Math.tan(angleToGoalRadians);

    SmartDashboard.putNumber("Distance", distance);
    return distance;
  }
  
}
  
