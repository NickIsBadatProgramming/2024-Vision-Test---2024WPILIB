// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.Vision;

public class SwerveChassis extends SubsystemBase {

  
  SwerveModule swerveFR, swerveFL, swerveBL, swerveBR;

  SwerveDriveKinematics kinematics;

  AHRS navx;

  PIDController xVelocity, yVelocity, aVelocity;

  Vision vision;

  
  SlewRateLimiter xLimiter, yLimiter, rLimiter;



  /** Creates a new swerveChassis. */
  public SwerveChassis() {
    
    this.swerveFR = RobotContainer.swerveFR;
    this.swerveFL = RobotContainer.swerveFL;
    this.swerveBL = RobotContainer.swerveBL;
    this.swerveBR = RobotContainer.swerveBR;

    this.navx = RobotContainer.navx;


    
    Translation2d m_frontRight = new Translation2d(swerveConstants.Wheelbase /2, -swerveConstants.Trackwidth/2); //Making 2D translations from the center of the robot to the swerve modules
    Translation2d m_frontLeft = new Translation2d(swerveConstants.Wheelbase /2, swerveConstants.Trackwidth/2);
    Translation2d m_backLeft = new Translation2d(-swerveConstants.Wheelbase /2, swerveConstants.Trackwidth/2);
    Translation2d m_backRight = new Translation2d(-swerveConstants.Wheelbase /2, -swerveConstants.Trackwidth/2);

    this.kinematics = new SwerveDriveKinematics(m_frontRight, m_frontLeft, m_backLeft, m_backRight);

    xLimiter = new SlewRateLimiter(swerveConstants.translationLimit * 2);
    yLimiter = new SlewRateLimiter(swerveConstants.translationLimit * 2);
    rLimiter = new SlewRateLimiter(swerveConstants.rotationLimit * 2);

    vision = RobotContainer.vision;



  }

  public void drive(double vy, double vx, double vr) {

    ChassisSpeeds speeds;

    if(RobotContainer.isUsingField){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vr, Rotation2d.fromDegrees(-navx.getYaw()));
    } else {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vr, Rotation2d.fromDegrees(0));
    }


    SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontRight = moduleStates[0];
    SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(frontRight, Rotation2d.fromDegrees(swerveFR.getModuleRotation()));
    SwerveModuleState frontLeft = moduleStates[1];
    SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft, Rotation2d.fromDegrees(swerveFL.getModuleRotation()));
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(backLeft, Rotation2d.fromDegrees(swerveBL.getModuleRotation()));
    SwerveModuleState backRight = moduleStates[3];
    SwerveModuleState backRightOptimized = SwerveModuleState.optimize(backRight, Rotation2d.fromDegrees(swerveBR.getModuleRotation()));

    swerveFR.driveModule(frontRightOptimized.speedMetersPerSecond, frontRightOptimized.angle.getDegrees());
    swerveFL.driveModule(-frontLeftOptimized.speedMetersPerSecond, frontLeftOptimized.angle.getDegrees());
    swerveBL.driveModule(-backLeftOptimized.speedMetersPerSecond, backLeftOptimized.angle.getDegrees());
    swerveBR.driveModule(backRightOptimized.speedMetersPerSecond, backRightOptimized.angle.getDegrees());

  }

  public void driveField(double vy, double vx, double vr) {

    ChassisSpeeds speeds;
    speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vy, vx, vr, Rotation2d.fromDegrees(0));


    SwerveModuleState[] moduleStates = this.kinematics.toSwerveModuleStates(speeds);

    SwerveModuleState frontRight = moduleStates[0];
    SwerveModuleState frontRightOptimized = SwerveModuleState.optimize(frontRight, Rotation2d.fromDegrees(swerveFR.getModuleRotation()));
    SwerveModuleState frontLeft = moduleStates[1];
    SwerveModuleState frontLeftOptimized = SwerveModuleState.optimize(frontLeft, Rotation2d.fromDegrees(swerveFL.getModuleRotation()));
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backLeftOptimized = SwerveModuleState.optimize(backLeft, Rotation2d.fromDegrees(swerveBL.getModuleRotation()));
    SwerveModuleState backRight = moduleStates[3];
    SwerveModuleState backRightOptimized = SwerveModuleState.optimize(backRight, Rotation2d.fromDegrees(swerveBR.getModuleRotation()));

    swerveFR.driveModule(frontRightOptimized.speedMetersPerSecond, frontRightOptimized.angle.getDegrees());
    swerveFL.driveModule(-frontLeftOptimized.speedMetersPerSecond, frontLeftOptimized.angle.getDegrees());
    swerveBL.driveModule(-backLeftOptimized.speedMetersPerSecond, backLeftOptimized.angle.getDegrees());
    swerveBR.driveModule(backRightOptimized.speedMetersPerSecond, backRightOptimized.angle.getDegrees());

  }

  public void resetPID() {
    xVelocity.reset();
    yVelocity.reset();
    aVelocity.reset();

    xLimiter.reset(0);
    yLimiter.reset(0);
    rLimiter.reset(0);
  }

  public double axisDeadzone(double input, double k) {
    if(Math.abs(input) >= k) {
      return input;
    }

    return 0;
  }

  







  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
