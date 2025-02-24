// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

   public final class ElevatorConstants{
    public static final int kElevatorManualControlAxis = 5; // todo: check that this is the right axis (via testing)

    public static final int kElevatorTestButtonID = 5;
    public static final double kElevatorMultiplier = 0.1;
    public static final double kTicksPerFoot = 13.9285;

    //Change later while testing robot
    public static final double kTicksPerSecondPerSpeed = 84.30841;

    public static final int deviceId = 19;
    public static final int deviceId2 = 24;

    public static final int limitSwitchDio = 0;


    // This is the max speed tolerated by trapezoidal. The unit is the speed that is put into setSpeed() (goes from -1 to 1)
    public static final double kMaxSpeed = 0.8;

    // This is the max acceleration tolerated by trapezoidal. Units are in ticks/s^2
    public static final double kMaxAcceleration = 150;

    public static final double kP = 0.05;
    public static final double kI = 0.007;
    public static final double kD = 0.0;
    //public static final double kffConstant = 0.1;

    public static final double kPositionDeadband  = 0.1;
    public static final double kVelocityDeadband = 0.05;
   }  


   // joystick constants removed -- see IOConstants.java (in constants folder)


    public static final class PivotConstants {
        public static final int kManualPivotControlAxis = 4;

        public static final int kPivotMotorID = 14;
        public static final int kPivotEncoderChannel = 7;

        public static final int kTopLimitSwitchID = 6;
        public static final int kBottomLimitSwitchID = 8;

        // sim constants
        public static final int kNumMotors = 2;
        public static final int kGearRatio = 112; // gear ratio is actually 112


        public static final double kPivotSpeed = 0.15;

        
        public static final double kPivotStowPosition = 0.677;
        public static final double kPivotIntakePosition = 0.997;

        public static final double kBottomPivotRotations = 0.998; // we need to be ON the hardstop
        // 5% limit subtracted from the original value
        public static final double kTopPivotRotations = 0.676 + 0.0167;

        public static final int kPivotStowButtonID = 1;
        public static final int kPivotIntakeButtonID = 2;

        public static final double kRotationTolerance = 0.02;

        // sim constants -- kp = 0.7, ki = 0.001, kd = 0.001
        public static final double kP = 3;
        public static final double kD = 0;
        public static final double kI = 1;

        public static final double kPivotPIDErrorTolerance = 0.001;
        public static final double kPivotDerivativeTolerance = 0.001;
    }



    public static final class IntakeConstants {
        public static final int kIntakeMotorID = 22;
        public static final int kIndexerMotorID = 17;

        // see IO Constants for exact button ID specifications (for the driver controller)
        /*
        public static final int kIntakeInButtonID = 3; //TODO: Change these button IDs
        public static final int kIntakeOutButtonID = 4;
        public static final int kIndexerInButtonID = 5;
        public static final int kIndexerOutButtonID = 6;
        public static final int kIntakeWithIndexerButtonID = 7;
        */

        public static final int kManualOuttakeTriggerAxis = 3;
    }
} 

