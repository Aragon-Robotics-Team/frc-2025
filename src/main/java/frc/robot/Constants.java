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
    public static final int kElevatorYAxis = 0;

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
   public final class JoystickConstants{
    public static final int kJoystickPort = 1;

   }
   
    public static final class IntakeConstants {
        public static final int kIntakeMotorID = 0;
        public static final double speed = .5;
        public static final int kIntakeButtonID = 1;
    }
    } 
 