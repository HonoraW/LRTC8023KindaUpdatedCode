// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Drive Train Constants 
  public static final class driveTrainConstants{
    public static final int leftFrontCANID = 3;
    public static final int leftBackCANID = 2;
    public static final int rightFrontCANID = 1;
    public static final int rightBackCANID = 4;
  }
  //put values in later for ks,kv,ka,kp from SysId
  public static final class DriveTrainConstants{
    public static final double ksVolts = 
    public static final double kvVoltSecondsPerMeter = 
    public static final double kaVoltSecondsSquaredPerMeter = 
    public static final double kpDriveVel = 
  }
  //horizontal distance between two wheels on a robot
  public static final double kTrackWidthMeters = Units.inchesToMeters(inches: 23);
  public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  public static final double kMaxSpeedMetersPerSecond = 3;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  //Reasonal baseline values for a RAMSETE follower in units of meters and seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  //get gear ratio from robot horizontal distance
  public static final double kGearRatio = 
  public static final double kWheelRadiusInches = //find wheel radius
  //really long equation he told me not to worry about aka "key" or conversion factor
  public static final double kLinearDistanceConversionFactor = (Units.inchesToMeters(1/(kGearRatio * 2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches)) * 10))
  // Intake Constants 
  public static final class intakeConstants{
    public static final int intakeMotor1 = 5;
    public static final int intakeMotor2 = 6;
  }
  // Operator Constants 
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
