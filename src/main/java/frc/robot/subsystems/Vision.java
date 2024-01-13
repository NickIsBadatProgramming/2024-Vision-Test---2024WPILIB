// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.autonomousConstants;

public class Vision extends SubsystemBase {

  private double tx, ty, tz, az;
  boolean target;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable visionTable;

  RollingAverage txAverage, tyAverage, tzAverage, azAverage;
  SlewRateLimiter txLimiter, tyLimiter, tzLimiter, azLimiter;

  /** Creates a new Vision. */
  public Vision() {
    visionTable = inst.getTable("limelight");

    txAverage = new RollingAverage(autonomousConstants.txAverage);
    tyAverage = new RollingAverage(autonomousConstants.tyAverage);
    tzAverage = new RollingAverage(autonomousConstants.tzAverage);
    azAverage = new RollingAverage(autonomousConstants.azAverage);
    
  }

  public double tx() {
    return this.tx - autonomousConstants.limelightXOffset;
  }

  public double ty() {
    return this.ty - autonomousConstants.limelightYOffset;
  }

  public double tz() {
    return this.tz - autonomousConstants.limelightZOffset;
  }

  public double az() {
    return this.az - autonomousConstants.limelightAZOffset;
  }

  public boolean target() {
    return this.target;
  }
  



  @Override
  public void periodic() {
    double[] array = visionTable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
    this.tx = txAverage.getAverage(array[0]);
    this.ty = tyAverage.getAverage(array[1]);
    this.tz = tzAverage.getAverage(array[2]);
    this.az = azAverage.getAverage(array[4]);

    this.target = (visionTable.getEntry("tv").getDouble(0) == 1);



    // SmartDashboard.putBoolean("Target Detected", isTarget());

    SmartDashboard.putNumber("X Translation", tx());
    SmartDashboard.putNumber("Y Translation", ty());
    SmartDashboard.putNumber("Z Translation", tz());
    SmartDashboard.putNumber("Z Angle", az());
    SmartDashboard.putBoolean("Target", target());
  }
}
