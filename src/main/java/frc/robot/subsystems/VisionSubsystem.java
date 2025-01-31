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

import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

//import edu.wpi.first.wpilibj.command.CommandBase;
import edu.wpi.first.math.controller.PIDController; 
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.LimelightHelpers;


public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  private static NetworkTable table;
  private static NetworkTableEntry tx;
  private static NetworkTableEntry ty;
  private static NetworkTableEntry ta;
/*
  private final double getTY = new getTY();
  private final double getTX = new getTX();
  private final double getArea = new getArea();
  */
  //private final Constants MAX_ROBOT_ANG_VEL_RAD_PER_SEC;

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

  public double limelightAimProportional(){

        

    double kp = 0.035;

    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kp;

    targetingAngularVelocity *= 1.0;

    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
   
  public double limelightRangeProportional(){
   double kp = .1;

   double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kp;

   targetingForwardSpeed *= 1.0;

   targetingForwardSpeed *= -1.0;
   
   return targetingForwardSpeed;
  }
  
}
  
