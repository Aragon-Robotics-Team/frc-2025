// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class DriveConstants {
    public static final double kMaxTranslationalMetersPerSecond = Units.feetToMeters(18.9); // used to be 18.9
    public static final double kMaxAngularAccelBotRotsPerSecondSquared = 3;
    public static final double kMaxTurningRadiansPerSecond = 1.5 * Math.PI;


    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3556 - 0.065, 0.3556 - 0.068),
            new Translation2d(0.3556 - 0.066, -0.3556 + 0.066),
            new Translation2d(0.3556 - 0.645, -0.3556 + 0.644),
            new Translation2d(0.3556 - 0.644, -0.3556 + 0.063));

    public static int kFrontLeftDriveId = 2;
    public static int kFrontLeftTurnId = 1;
    public static int kFrontLeftAbsoluteEncoderPort = 0;
    public static double kFrontLeftAbsoluteEncoderOffset = -0.529990463821544 - (Math.PI / 2.0);
    public static boolean kFrontLeftDriveReversed = false;
    public static boolean kFrontLeftTurningReversed = true;
        
    public static int kFrontRightDriveId = 4;
    public static int kFrontRightTurnId = 3;
    public static int kFrontRightAbsoluteEncoderPort = 1;
    public static double kFrontRightAbsoluteEncoderOffset = 1.415447275814819 + (Math.PI / 2.0);
    public static boolean kFrontRightDriveReversed = true;
    public static boolean kFrontRightTurningReversed = true;
        
    public static int kBackLeftDriveId = 8;
    public static int kBackLeftTurnId = 7;
    public static int kBackLeftAbsoluteEncoderPort = 3;
    public static double kBackLeftAbsoluteEncoderOffset = 1.253910661599266 + (Math.PI / 2.0);
    public static boolean kBackLeftDriveReversed = false;
    public static boolean kBackLeftTurningReversed = true;
        
    public static int kBackRightDriveId = 6;
    public static int kBackRightTurnId = 5;
    public static int kBackRightAbsoluteEncoderPort = 2;
    public static double kBackRightAbsoluteEncoderOffset = -0.456511594330128 - (Math.PI / 2.0);
    public static boolean kBackRightDriveReversed = true;
    public static boolean kBackRightTurningReversed = true;

    public static PIDConstants kTranslationConstants = new PIDConstants(0.75);
    public static PIDConstants kRotationConstants = new PIDConstants(0.75);
}