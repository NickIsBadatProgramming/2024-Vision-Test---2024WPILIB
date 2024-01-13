// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveConstants;

public class SwerveModule extends SubsystemBase {

  
  //Drive motors and encoder
  protected TalonFX rotationMotor, driveMotor;
  protected CANcoder rotationEncoder;
  
  double driveMotorSpeed, steerMotorSpeed, lastAngle;


  /** Creates a new swerveModule. */
  public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANcoder rotationEncoder) {
    this.driveMotor = driveMotor;
    this.rotationMotor = rotationMotor;
    this.rotationEncoder = rotationEncoder;
    this.driveMotorSpeed = 0; //Just for emergency's sake
    this.steerMotorSpeed = 0;
    this.lastAngle = 0;
  }

  public void driveModule(double speed, double angle) {

    lastAngle = angle;

    double difference = angle - this.rotationEncoder.getAbsolutePosition().getValueAsDouble();

    if(difference > 180) {
      difference -= 360;
    }
    if(difference < -180) {
      difference += 360;
    }


    this.steerMotorSpeed = steerSpeed(rotationEncoder.getAbsolutePosition().getValueAsDouble(), lastAngle);
    this.driveMotorSpeed = speed * ((90 - Math.abs(difference))/90); //This won't ever be above 90 because of the way the swerve library works



    updateMotorSpeeds();
  }

  public double steerSpeed(double currentAngle, double desiredAngle) {
    //at 0 degrees from the angle we want he rotation speed to be 0
    // at 180 degrees we have a constant in constants for our maximum speed
    //these two points can make a linear graph

    double difference = desiredAngle - currentAngle;
    if(difference > 180) {
      difference -= 360;
    }
    if(difference < -180) {
      difference += 360;
    }

    if(Math.abs(difference) < swerveConstants.angleTolerance) {
      return 0;
    }

    return swerveConstants.additionalTurnSpeed * (difference/180) + (swerveConstants.minModuleTurnSpeed * (difference/Math.abs(difference)));

  }

  public void updateMotorSpeeds() {
    this.driveMotor.set(driveMotorSpeed);
    this.rotationMotor.set(steerMotorSpeed);
  }

  public CANcoder getCANCoder() {
    return this.rotationEncoder;
  }

  public TalonFX getDriveMotor() {
    return this.driveMotor;
  }

  public TalonFX getRotationMotor() {
    return this.rotationMotor;
  }

  public double getModuleRotation() {
    return this.rotationEncoder.getAbsolutePosition().getValueAsDouble() * 360;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
