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
    // These are used in SwerveJoystick
    // NEVER CHANGE THESE
    public static final double kMaxTranslationalMetersPerSecond = 5;
    public static final double kMaxTurningRadiansPerSecond = 1.35 * Math.PI;
    public static final double kTranslationalDeadbandMetersPerSecond = 0.01;

    public static final PIDConstants kTranslationConstants = new PIDConstants(0.75); //PID constants for whole robot chassis speeds
    public static final PIDConstants kRotationConstants = new PIDConstants(0.75);

    //ALL OF THESE WILL HAVE TO BE DIFFERENT WITH THE NEW DRIVETRAIN

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(+0.295, +0.292),
        new Translation2d(+0.295, -0.292),
        new Translation2d(-0.295, +0.292),
        new Translation2d(-0.295, -0.292));

    public static final int kFrontLeftDriveId = 1; //CAN
    public static final int kFrontLeftTurnId = 11; //CAN
    public static final int kFrontLeftTurnEncoderPort = 1; //PWM
    public static final double kFrontLeftTurnEncoderOffset = 0.58 + 1.57;
    public static final boolean kFrontLeftDriveReversed = true;
    public static final boolean kFrontLeftTurningReversed = true;
        
    public static final int kFrontRightDriveId = 4; //CAN
    public static final int kFrontRightTurnId = 14; //CAN
    public static final int kFrontRightTurnEncoderPort = 4; //PWM
    public static final double kFrontRightTurnEncoderOffset = 5.91 + 1.54;
    public static final boolean kFrontRightDriveReversed = true;
    public static final boolean kFrontRightTurningReversed = true;
        
    public static final int kBackLeftDriveId = 2; //CAN
    public static final int kBackLeftTurnId = 12; //CAN
    public static final int kBackLeftTurnEncoderPort = 2; //PWM
    public static final double kBackLeftTurnEncoderOffset = 1.8 + 1.59;
    public static final boolean kBackLeftDriveReversed = true;
    public static final boolean kBackLeftTurningReversed = false;
        
    public static final int kBackRightDriveId = 3; //CAN
    public static final int kBackRightTurnId = 13; //CAN
    public static final int kBackRightTurnEncoderPort = 3; //PWM
    public static final double kBackRightTurnEncoderOffset = 2.46 + 1.56;
    public static final boolean kBackRightDriveReversed = true;
    public static final boolean kBackRightTurningReversed = false;


    public static final PIDController kModuleDrivePIDConstants = new PIDController(0.17, 1.7, 0.0); //PID constants for the drive and steer of each module
    public static final PIDController kModuleTurnPIDConstants = new PIDController(0.5, 0.0, 0.0);


    // VERY IMPORTANT NUMBERS BELOW YOU GOTTA CHANGE THEM;

    public static final double kDriveGearRatio = 6.122;
    public static final double kWheelCircumference = Math.PI * Units.inchesToMeters(4);
    public static final double kDriveSensorToMechanismRatio = kDriveGearRatio/kWheelCircumference; //how much distance is traveled for one rotation on the drive TalonFX

    public static final double kTurnEncoderPositionToRadians = Math.PI * 2; //since turn encoder is right on the output shaft, we only need to convert rotations into radians




}