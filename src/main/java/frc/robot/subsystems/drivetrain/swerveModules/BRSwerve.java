// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModules;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivetrain.SwerveModule;

public class BRSwerve extends SwerveModule {
  /** Creates a new FRSwerve. */
  public BRSwerve(TalonFX driveMotor, TalonFX rotationMotor, CANcoder rotationEncoder) {
    super(driveMotor, rotationMotor, rotationEncoder);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Back Right Raw Encoder Value", this.getModuleRotation());
  }
}
