// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YAGSLSwerveDrive extends SubsystemBase {
  /** Creates a new YAGSLSwerveDrive. */
  SwerveDriveKinematics kinematics;
  SwerveDriveOdometry odometry;
  Canandgyro gyro;
  SwerveModule[] swerveModules;

  public YAGSLSwerveDrive() {
    swerveModules = new SwerveModule[4];
    kinematics = new SwerveDriveKinematics(
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
      new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0))
    );
    gyro = new Canandgyro(0);
    odometry = new SwerveDriveOdometry(
      kinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
      new Pose2d(0, 0, new Rotation2d())
    );
  }

  public void drive() {
    ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(0), Units.degreesToRadians(0), Units.degreesToRadians(0));
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(testSpeeds);
    swerveModules[0].setDesiredState(swerveModuleStates[0]);
    swerveModules[1].setDesiredState(swerveModuleStates[1]);
    swerveModules[2].setDesiredState(swerveModuleStates[2]);
    swerveModules[3].setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    return new SwerveModulePosition[]{
      new SwerveModulePosition(swerveModules[0].getDrivePosition(), swerveModules[0].getRotation()), // Front-Left
      new SwerveModulePosition(swerveModules[1].getDrivePosition(), swerveModules[1].getRotation()), // Front-Right
      new SwerveModulePosition(swerveModules[2].getDrivePosition(), swerveModules[2].getRotation()), // Back-Left
      new SwerveModulePosition(swerveModules[3].getDrivePosition(), swerveModules[3].getRotation())
    };
  }

  @Override
  public void periodic() {
    odometry.update(gyro.getRotation2d(), getCurrentSwerveModulePositions());
  }
}
