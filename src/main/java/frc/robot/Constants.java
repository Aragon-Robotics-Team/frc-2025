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
    public static final class PivotConstants {
        public static final int kLeftPivotMotorID = 0; 
        public static final int kRightPivotMotorID = 1; //TODO: change

        public static final int kNumMotors = 2;
        public static final int kGearRatio = 108;

        public static final int kEncoderChannelA = 0;
        public static final int kEncoderChannelB = 1;

        public static final double kPivotSpeed = 0.7;

        public static final double kPivotPositionToA = 0;
        public static final double kPivotPositionToB = 2;
        public static final double kPivotPositionToC = 10;

        public static final int kButtonNumToA = 1;
        public static final int kButtonNumToB = 2;
        public static final int kButtonNumToC = 3;

        public static final double kRotationTolerance = 0.05;
    }

   public final class ElevatorConstants{
    public static final int kElevatorYAxis = 0;
    public static final double kElevatorMultiplier = 0.1;
    public static final int deviceId = 19;
    public static final int deviceId2=24;
    public static final int limitSwitchDio=0;


    public static final double kElevatorTickConversionRate = 13.9285; // to move our elevator a foot upwards, it costs 13.9285 elevator ticks (and two trainees)
    // max ticks -- ~60.25

   }  
   public final class JoystickConstants{
    public static final int kJoystickPort = 1;

   }
   

}
