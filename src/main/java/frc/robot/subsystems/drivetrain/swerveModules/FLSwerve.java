// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModules;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class FLSwerve extends SwerveModule {
  /** Creates a new FLSwerve. */
  public FLSwerve(TalonFX driveMotor, TalonFX rotationMotor, CANcoder rotationEncoder) {
    super(driveMotor, rotationMotor, rotationEncoder);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Front Left Raw Encoder Value", this.getModuleRotation());
  }
}