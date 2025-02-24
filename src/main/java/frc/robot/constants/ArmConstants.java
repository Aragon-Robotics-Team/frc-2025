// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class ArmConstants {
  public static final int kArmTalonDeviceId = 23; 
  public static final int kTopLimitSwitchChannel = 1;
  public static final int kBottomLimitSwitchChannel = 10;
  public static final int kArmManualControlAxis = 1;
  public static final double kArmMultiplier = 0.5;

  public static final double kP = 1;
  public static final double kI = 0.01;
  public static final double kD = 0;

  public static final int kEncoderChannel = 5; 
  public static final int ArmOuttakeMotorDeviceId = 18;
  public static final int kArmOuttakeIntakeButtonID = 5;
  public static final int kArmOuttakeOuttakeButtonID = 6;
  
  public static int encoderChannel = 9; 
  // DIO channel 0 has been taken by elevator
  // DIO channels 1, 2, 3, 4 have been taken by the swerve

  public static final int kArmButtonID = 1;
}
// find in future!