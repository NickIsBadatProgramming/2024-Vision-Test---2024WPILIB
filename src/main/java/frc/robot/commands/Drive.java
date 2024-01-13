// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.swerveConstants;

public class Drive extends Command {


  double vx, vy, vr;
  XboxController driveController;
  double rightTrigger;

  SlewRateLimiter xLimiter, yLimiter, rLimiter;


  /** Creates a new drive. */
  public Drive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerve);
  }

  public double axisDeadzone(double input) {
    if(Math.abs(input) >= swerveConstants.xboxDeadzone) {
      return input;
    }

    return 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.swerve.drive(0, 0, 0);
    
    this.driveController = RobotContainer.driveController;
    this.xLimiter = new SlewRateLimiter(swerveConstants.translationLimit);
    this.yLimiter = new SlewRateLimiter(swerveConstants.translationLimit);
    this.rLimiter = new SlewRateLimiter(swerveConstants.rotationLimit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    rightTrigger =  (0.7 * driveController.getRightTriggerAxis()) + 0.3;

    vx = rightTrigger * xLimiter.calculate(axisDeadzone(-driveController.getRawAxis(0)));

    vy = rightTrigger * yLimiter.calculate(axisDeadzone(-driveController.getRawAxis(1)));

    vr = rightTrigger * rLimiter.calculate(axisDeadzone(-driveController.getRawAxis(4)));

    SmartDashboard.putNumber("X velocity", vx);
    SmartDashboard.putNumber("Y Velocity", vy);
    SmartDashboard.putNumber("R Velocity", vr);
    SmartDashboard.putNumber("Left Trigger", rightTrigger);


    RobotContainer.swerve.drive(vy, vx, vr);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
