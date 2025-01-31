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
  public static class PivotConstants {
    public static final int kPivotMotorID = 0;

    public static final double kArcadePivotSpeedLimiter = 1.0;

    public static final double kToleranceRotations = 0.1; // within 0.1 of a rotation is when we're done
    public static final double kPivotSpeed = 0.8;

    public static final double kGearRatio = 10.0;


    public static final double kP = 0.4;
  }

  public static class JoystickConstants{
    public static final int kPort = 0;
  }
}
