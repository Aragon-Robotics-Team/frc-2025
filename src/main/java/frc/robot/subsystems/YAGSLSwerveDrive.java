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
import frc.robot.constants.DriveConstants;

public class YAGSLSwerveDrive extends SubsystemBase {
  /** Creates a new YAGSLSwerveDrive. */
  SwerveDriveKinematics m_kinematics;
  SwerveDriveOdometry m_odometry;
  Canandgyro m_gyro;
  SwerveModule[] m_swerveModules;

  public YAGSLSwerveDrive() {
    m_swerveModules = new SwerveModule[4];
    m_kinematics = new SwerveDriveKinematics(
      new Translation2d(+0.295, +0.295),
      new Translation2d(+0.295, -0.295),
      new Translation2d(-0.295, +0.295),
      new Translation2d(-0.295, -0.295)
    );
    m_gyro = new Canandgyro(DriveConstants.kIMUCanID);
    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      m_gyro.getRotation2d(),
      new SwerveModulePosition[]{new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
      new Pose2d(0, 0, new Rotation2d())
    );
  }

  public void drive() {
    ChassisSpeeds testSpeeds = new ChassisSpeeds(Units.inchesToMeters(0), Units.degreesToRadians(0), Units.degreesToRadians(0));
    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(testSpeeds);
    m_swerveModules[0].setDesiredState(swerveModuleStates[0]);
    m_swerveModules[1].setDesiredState(swerveModuleStates[1]);
    m_swerveModules[2].setDesiredState(swerveModuleStates[2]);
    m_swerveModules[3].setDesiredState(swerveModuleStates[3]);
  }

  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    return new SwerveModulePosition[]{
      new SwerveModulePosition(m_swerveModules[0].getDrivePosition(), m_swerveModules[0].getRotation()), // Front-Left
      new SwerveModulePosition(m_swerveModules[1].getDrivePosition(), m_swerveModules[1].getRotation()), // Front-Right
      new SwerveModulePosition(m_swerveModules[2].getDrivePosition(), m_swerveModules[2].getRotation()), // Back-Left
      new SwerveModulePosition(m_swerveModules[3].getDrivePosition(), m_swerveModules[3].getRotation())
    };
  }

  public void stop() {
    m_swerveModules[0].stop();
    m_swerveModules[1].stop();
    m_swerveModules[2].stop();
    m_swerveModules[3].stop();
  }

  @Override
  public void periodic() {
    m_odometry.update(m_gyro.getRotation2d(), getCurrentSwerveModulePositions());
  }
}
