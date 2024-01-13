// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class swerveConstants {
    
      //Driving constants
      public final static double swerveP = 0.001;
      public final static double swerveI = 0.0001;
      public final static double swerveD = 0.0000000000001;

      public final static double FROffset = 359.8;
      public final static double FLOffset = 164.7;
      public final static double BLOffset = 180.8;
      public final static double BROffset = 212.5;

      public final static double xboxDeadzone = 0.15;
      public final static double translationLimit = 3;
      public final static double rotationLimit = 2;

      public final static double angleTolerance = 1;
      public static final double minModuleTurnSpeed = 0.02;
      public static final double additionalTurnSpeed = 0.28;



      //Dimensional constants

      public final static double Trackwidth = 0.4445; //Meters
      public final static double Wheelbase = 0.596955;

  }

  public final class autonomousConstants {

    public static final double limelightXOffset = 0.16;
    public static final double limelightYOffset = -0.11;
    public static final double limelightZOffset = -0.79;
    public static final double limelightAZOffset = -0.1;


    public static final int txAverage = 10;
    public static final int tyAverage = 10;
    public static final int tzAverage = 10;
    public static final int azAverage = 5;

  }

}
