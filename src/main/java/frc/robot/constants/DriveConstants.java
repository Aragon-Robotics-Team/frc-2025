// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class DriveConstants {
    public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(18.9); //should probably reduce these
    public static final double kMaxAngularAccelBotRotsPerSecondSquared = 3;
    public static final double kMaxTurningRadiansPerSecond = 1.5 * Math.PI;
    public static final double kTranslationalDeadbandMetersPerSecond = 0.01;

    public static final PIDConstants kTranslationConstants = new PIDConstants(0.75); //PID constants for whole robot chassis speeds
    public static final PIDConstants kRotationConstants = new PIDConstants(0.75);

    //ALL OF THESE WILL HAVE TO BE DIFFERENT WITH THE NEW DRIVETRAIN

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
            new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
            new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
            new Translation2d(0.3556 - 0.644, -0.3556 + 0.063));

    public static final int kFrontLeftDriveId = 2; //CAN
    public static final int kFrontLeftTurnId = 1; //CAN
    public static final int kFrontLeftTurnEncoderPort = 0; //PWM
    public static final double kFrontLeftTurnEncoderOffset = -0.529990463821544 - (Math.PI / 2.0);
    public static final boolean kFrontLeftDriveReversed = false;
    public static final boolean kFrontLeftTurningReversed = true;
        
    public static final int kFrontRightDriveId = 4; //CAN
    public static final int kFrontRightTurnId = 3; //CAN
    public static final int kFrontRightTurnEncoderPort = 1; //PWM
    public static final double kFrontRightTurnEncoderOffset = 1.415447275814819 + (Math.PI / 2.0);
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kFrontRightTurningReversed = true;
        
    public static final int kBackLeftDriveId = 8; //CAN
    public static final int kBackLeftTurnId = 7; //CAN
    public static final int kBackLeftTurnEncoderPort = 3; //PWM
    public static final double kBackLeftTurnEncoderOffset = 1.253910661599266 + (Math.PI / 2.0);
    public static final boolean kBackLeftDriveReversed = false;
    public static final boolean kBackLeftTurningReversed = true;
        
    public static final int kBackRightDriveId = 6; //CAN
    public static final int kBackRightTurnId = 5; //CAN
    public static final int kBackRightTurnEncoderPort = 2; //PWM
    public static final double kBackRightTurnEncoderOffset = -0.456511594330128 - (Math.PI / 2.0);
    public static final boolean kBackRightDriveReversed = true;
    public static final boolean kBackRightTurningReversed = true;


    public static final PIDController kModuleDrivePIDConstants = new PIDController(0.17, 1.7, 0.0); //PID constants for the drive and steer of each module
    public static final PIDController kModuleTurnPIDConstants = new PIDController(0.4, 0.0, 0.0);


    // VERY IMPORTANT NUMBERS BELOW

    public static final double kDriveGearRatio = 5.9;
    public static final double kWheelCircumference = Math.PI * Units.inchesToMeters(4);
    public static final double kDriveSensorToMechanismRatio = kDriveGearRatio/kWheelCircumference; //how much distance is traveled for one rotation on the drive TalonFX

    public static final double kTurnEncoderPositionToRadians = Math.PI * 2; //since turn encoder is right on the output shaft, we only need to convert rotations into radians




}