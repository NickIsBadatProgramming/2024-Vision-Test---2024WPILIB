// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.swerveConstants;
import frc.robot.commands.CenterToTag;
import frc.robot.commands.Drive;
import frc.robot.commands.ResetFeild;
import frc.robot.commands.UseField;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drivetrain.SwerveChassis;
import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.subsystems.drivetrain.swerveModules.BLSwerve;
import frc.robot.subsystems.drivetrain.swerveModules.BRSwerve;
import frc.robot.subsystems.drivetrain.swerveModules.FLSwerve;
import frc.robot.subsystems.drivetrain.swerveModules.FRSwerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public static TalonFX dFR, dFL, dBL, dBR;
  public static TalonFX sFR, sFL, sBL, sBR;
  public static CANcoder FR, FL, BL, BR;

  public static SwerveModule swerveFR, swerveFL, swerveBL, swerveBR;
  public static SwerveChassis swerve;

  public static AHRS navx;

  public static XboxController driveController;
  public static JoystickButton xbox1SS; //Screen Share button, two rectangles
  public static JoystickButton xbox1Settings;
  public static JoystickButton xbox1LeftJoystickButton;

  public static Drive drive;
  public static Boolean isUsingField = true;
  ResetFeild resetFeild;
  UseField useField;

  public static Vision vision;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


    //Drive Motors
    dFR = new TalonFX(1);
    dFL = new TalonFX(3);
    dBL = new TalonFX(5);
    dBR = new TalonFX(7);

    //Steer Motors
    sFR = new TalonFX(2);
    sFL = new TalonFX(4);
    sBL = new TalonFX(6);
    sBR = new TalonFX(8);

    MotorOutputConfigs talonConfigs = new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake);


    sFR.getConfigurator().apply(talonConfigs);
    sBL.getConfigurator().apply(talonConfigs);
    sBR.getConfigurator().apply(talonConfigs);
    sFL.getConfigurator().apply(talonConfigs);
    dFR.getConfigurator().apply(talonConfigs);
    dFL.getConfigurator().apply(talonConfigs);
    dBL.getConfigurator().apply(talonConfigs);
    dBR.getConfigurator().apply(talonConfigs);

    dFR.getConfigurator().refresh(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    dBR.getConfigurator().refresh(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    //CANCoders
    FR = new CANcoder(12);
    FL = new CANcoder(9);
    BL = new CANcoder(10);
    BR = new CANcoder(11);

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
      .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)); //TODO get this back from 0 - 360
      

    FR.getConfigurator().apply(cancoderConfiguration);  
    BR.getConfigurator().apply(cancoderConfiguration);
    FL.getConfigurator().apply(cancoderConfiguration);
    BL.getConfigurator().apply(cancoderConfiguration);

    
    FR.getConfigurator().refresh(new MagnetSensorConfigs().withMagnetOffset(swerveConstants.FROffset/360));
    FL.getConfigurator().refresh(new MagnetSensorConfigs().withMagnetOffset(swerveConstants.FLOffset/360));
    BL.getConfigurator().refresh(new MagnetSensorConfigs().withMagnetOffset(swerveConstants.BLOffset/360));
    BR.getConfigurator().refresh(new MagnetSensorConfigs().withMagnetOffset(swerveConstants.BROffset/360));

    swerveFR = new FRSwerve(dFR, sFR, FR);
    swerveFL = new FLSwerve(dFL, sFL, FL);
    swerveBL = new BLSwerve(dBL, sBL, BL);
    swerveBR = new BRSwerve(dBR, sBR, BR);

    navx = new AHRS(SerialPort.Port.kMXP);
    vision = new Vision();
    swerve = new SwerveChassis();
    drive = new Drive();

    driveController = new XboxController(0);
    xbox1SS = new JoystickButton(driveController,7);
    xbox1Settings = new JoystickButton(driveController,8);
    xbox1LeftJoystickButton = new JoystickButton(driveController, 9);

    resetFeild = new ResetFeild();
    useField = new UseField();


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    swerve.setDefaultCommand(drive);

    xbox1SS.onTrue(resetFeild);
    xbox1Settings.onTrue(useField);
    xbox1LeftJoystickButton.toggleOnTrue(new CenterToTag());

  

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; //TODO add something here
  }
}
