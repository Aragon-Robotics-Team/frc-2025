package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

public final class VisionConstants {
    // 6.47 in between each Tag and pole left/ right to it
    public static final int kTag6Angle = 120; //TODO: Change these
    public static final int kTag7Angle = 180;
    public static final int kTag8Angle = -120;
    public static final int kTag9Angle = -60;
    public static final int kTag10Angle = 0;
    public static final int kTag11Angle = 60;
    public static final int kTag17Angle = -120;
    public static final int kTag18Angle = 180;
    public static final int kTag19Angle = 120;
    public static final int kTag20Angle = 60;
    public static final int kTag21Angle = 0;
    public static final int kTag22Angle = -60;

    // Red
    public static final Pose2d kTag6Left = new Pose2d(new Translation2d(524.887*0.0254, 126.935*0.0254), new Rotation2d(Math.toRadians(kTag6Angle)));
    public static final Pose2d kTag6Right = new Pose2d(new Translation2d(536.093*0.0254, 133.405*0.0254), new Rotation2d(Math.toRadians(kTag6Angle)));
    
    public static final Pose2d kTag7Left = new Pose2d(new Translation2d(546.87*0.0254, 152.03*0.0254), new Rotation2d(Math.toRadians(kTag7Angle)));
    public static final Pose2d kTag7Right = new Pose2d(new Translation2d(546.87*0.0254, 164.97*0.0254), new Rotation2d(Math.toRadians(kTag7Angle)));
    
    public static final Pose2d kTag8Left = new Pose2d(new Translation2d(536.093*0.0254, 183.595*0.0254), new Rotation2d(Math.toRadians(kTag8Angle)));
    public static final Pose2d kTag8Right = new Pose2d(new Translation2d(524.887*0.0254, 190.065*0.0254), new Rotation2d(Math.toRadians(kTag8Angle)));
    
    public static final Pose2d kTag9Left = new Pose2d(new Translation2d(503.373*0.0254, 190.065*0.0254), new Rotation2d(Math.toRadians(kTag9Angle)));
    public static final Pose2d kTag9Right = new Pose2d(new Translation2d(492.167*0.0254, 183.595*0.0254), new Rotation2d(Math.toRadians(kTag9Angle)));
    
    public static final Pose2d kTag10Left = new Pose2d(new Translation2d(481.39*0.0254, 164.97*0.0254), new Rotation2d(Math.toRadians(kTag10Angle)));
    public static final Pose2d kTag10Right = new Pose2d(new Translation2d(481.39*0.0254, 152.03*0.0254), new Rotation2d(Math.toRadians(kTag10Angle)));
    
    public static final Pose2d kTag11Left = new Pose2d(new Translation2d(492.167*0.0254, 133.405*0.0254), new Rotation2d(Math.toRadians(kTag11Angle)));
    public static final Pose2d kTag11Right = new Pose2d(new Translation2d(503.373*0.0254, 126.935*0.0254), new Rotation2d(Math.toRadians(kTag11Angle)));

    // Blue
    public static final Pose2d kTag17Left = new Pose2d(new Translation2d(154.787*0.0254, 133.405*0.0254), new Rotation2d(Math.toRadians(kTag17Angle)));
    public static final Pose2d kTag17Right = new Pose2d(new Translation2d(165.993*0.0254, 126.935*0.0254), new Rotation2d(Math.toRadians(kTag17Angle)));
    
    public static final Pose2d kTag18Left = new Pose2d(new Translation2d(144.00*0.0254, 164.97*0.0254), new Rotation2d(Math.toRadians(kTag18Angle)));
    public static final Pose2d kTag18Right = new Pose2d(new Translation2d(144.00*0.0254, 152.03*0.0254), new Rotation2d(Math.toRadians(kTag18Angle)));
    
    public static final Pose2d kTag19Left = new Pose2d(new Translation2d(165.993*0.0254, 190.065*0.0254), new Rotation2d(Math.toRadians(kTag19Angle)));
    public static final Pose2d kTag19Right = new Pose2d(new Translation2d(154.787*0.0254, 183.595*0.0254), new Rotation2d(Math.toRadians(kTag19Angle)));
    
    public static final Pose2d kTag20Left = new Pose2d(new Translation2d(198.703*0.0254, 183.595*0.0254), new Rotation2d(Math.toRadians(kTag20Angle)));
    public static final Pose2d kTag20Right = new Pose2d(new Translation2d(187.497*0.0254, 190.065*0.0254), new Rotation2d(Math.toRadians(kTag20Angle)));
    
    public static final Pose2d kTag21Left = new Pose2d(new Translation2d(209.49*0.0254, 152.03*0.0254), new Rotation2d(Math.toRadians(kTag21Angle)));
    public static final Pose2d kTag21Right = new Pose2d(new Translation2d(209.49*0.0254, 164.47*0.0254), new Rotation2d(Math.toRadians(kTag21Angle)));
    
    public static final Pose2d kTag22Left = new Pose2d(new Translation2d(187.497*0.0254, 126.935*0.0254), new Rotation2d(Math.toRadians(kTag22Angle)));
    public static final Pose2d kTag22Right = new Pose2d(new Translation2d(198.703*0.0254, 133.405*0.0254), new Rotation2d(Math.toRadians(kTag22Angle)));


    public static final int[] kTagAngles = {kTag6Angle, kTag7Angle, kTag8Angle, kTag9Angle, kTag10Angle, kTag11Angle, 0, 0, 0, 0, 0, kTag17Angle, kTag18Angle, kTag19Angle, kTag20Angle, kTag21Angle, kTag22Angle};
}
