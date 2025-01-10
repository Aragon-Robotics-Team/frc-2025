// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.fasterxml.jackson.databind.util.Named;

import com.reduxrobotics.sensors.canandgyro.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.swerve.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.OIConstants;

public class SwerveDrive extends SubsystemBase 
{
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveId, 
    DriveConstants.kFrontLeftTurnId,
    DriveConstants.kFrontLeftAbsoluteEncoderPort, 
    DriveConstants.kFrontLeftAbsoluteEncoderOffset, 
    DriveConstants.kFrontLeftDriveReversed,
    DriveConstants.kFrontLeftTurningReversed, 
    0
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveId, 
    DriveConstants.kFrontRightTurnId,
    DriveConstants.kFrontRightAbsoluteEncoderPort, 
    DriveConstants.kFrontRightAbsoluteEncoderOffset, 
    DriveConstants.kFrontRightDriveReversed, 
    DriveConstants.kFrontRightTurningReversed, 
    1
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveId, 
    DriveConstants.kBackLeftTurnId,
    DriveConstants.kBackLeftAbsoluteEncoderPort, 
    DriveConstants.kBackLeftAbsoluteEncoderOffset, 
    DriveConstants.kBackLeftDriveReversed, 
    DriveConstants.kBackLeftTurningReversed, 
    2
  );

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveId, 
    DriveConstants.kBackRightTurnId,
    DriveConstants.kBackRightAbsoluteEncoderPort, 
    DriveConstants.kBackRightAbsoluteEncoderOffset, 
    DriveConstants.kBackRightDriveReversed, 
    DriveConstants.kBackRightTurningReversed, 
    3
  );

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeft.getTranslation(), m_frontRight.getTranslation(), m_backLeft.getTranslation(), m_backRight.getTranslation());

  private final Canandgyro m_imu = new Canandgyro(OIConstants.kIMUCanID);

  private double m_totalCurrent;

  private final Field2d m_field = new Field2d();

  private final SwerveDriveOdometry m_odo = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  });

  private double m_xStartPose;
  private double m_yStartPose;

  public void resetHeading() {
    m_imu.setYaw(0.0);
  }

  public InstantCommand resetHeadingCommand(){
    return new InstantCommand(this::resetHeading, this);
  }

  public void adjustAngle(double angle){
    m_imu.setYaw(angle);
  }

  public void resetOdo(Pose2d pose) {
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    }, pose);
  }

  public Pose2d getPoseMeters(){
    return m_odo.getPoseMeters();
  }

  public SwerveDriveKinematics getSwerveKinematics(){
    return DriveConstants.kDriveKinematics;
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kMaxTranslationalMetersPerSecond);

    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  Rotation2d degree = new Rotation2d(Math.PI/2);
  SwerveModuleState frontLeft = new SwerveModuleState(3.0, degree);
  SwerveModuleState frontRight = new SwerveModuleState(3.0, degree);
  SwerveModuleState backLeft = new SwerveModuleState(3.0, degree);
  SwerveModuleState backRight = new SwerveModuleState(3.0, degree);
  SwerveModuleState[] testStates = {frontLeft, frontRight, backLeft, backRight};

  public void setSwerveModuleStatesAuto(SwerveModuleState[] states) {
    setModuleStates(states);  
  }

  public void resetAllDistances() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_backLeft.resetEncoders();
    m_backRight.resetEncoders();
  }

  public void printVelocitiesandPositions(){
    System.out.println("Front left Velocity: " + m_frontLeft.getDriveVelocity());
    System.out.println("Front left Position: " + m_frontLeft.getDrivePosition());
    System.out.println("Front right velocity: " + m_frontRight.getDriveVelocity());
    System.out.println("Front right Position: " + m_frontRight.getDrivePosition());
    System.out.println("Back left velocity: " + m_backLeft.getDriveVelocity());
    System.out.println("Back left Position: " + m_backLeft.getDrivePosition());
    System.out.println("Back right velocity: " + m_backRight.getDriveVelocity());
    System.out.println("Back right Position: " + m_backRight.getDrivePosition());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(-m_imu.getYaw(), 360));
  }

  public ChassisSpeeds getChassisSpeeds() { 
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(), 
      m_frontRight.getState(), 
      m_backLeft.getState(), 
      m_backRight.getState()
    );
  }

 
  public double getFrontLeftRotation()

  {
    return m_frontLeft.getRotationActual();
  }

  SwerveModule[] m_modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  public String[] m_moduleNames = {"frontLeft", "frontRight", "backLeft", "backRight"};
  
  public interface I
  {
    public double get(SwerveModule module);

  }
  public double[] getFromAllSwerveModules(I property)
  {
    double[] properties = new double[m_modules.length];
    for(int i = 0; i < properties.length; i++)
    {
      properties[i] = property.get(m_modules[i]);
    }
    return properties;

  }

  public void driveRobotRelative(ChassisSpeeds speeds) { 
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void resetOdoToPose(){
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      }, new Pose2d(m_xStartPose, m_yStartPose, getAngle()));
  }

  public SwerveDrive(Command stowed, Command autoShoot, Command autoIntake, Command groundIntake, Command outtake, Command subwoofer, Command rightUnderStage) 
  {
    NamedCommands.registerCommand("Print", new PrintCommand("Print command is running!!!"));
    NamedCommands.registerCommand("Stow", stowed);
    NamedCommands.registerCommand("AutoShoot", autoShoot);
    NamedCommands.registerCommand("AutoIntake", autoIntake);
    NamedCommands.registerCommand("GroundIntake", groundIntake);
    NamedCommands.registerCommand("Outtake", outtake);
    NamedCommands.registerCommand("Subwoofer", subwoofer);
    NamedCommands.registerCommand("RightUnderStage", rightUnderStage);
    
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
    };
    AutoBuilder.configure
    (
      this::getPoseMeters,
      this::resetOdo,
        this::getChassisSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(DriveConstants.kTranslationConstants, DriveConstants.kRotationConstants), 
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
      },
      this
    );

    SmartDashboard.putData("Swerve/Distance/reset", new InstantCommand(this::resetAllDistances));
    double m_angle = SmartDashboard.getNumber("Driving/Adjust angle", 0);
    SmartDashboard.putNumber("Driving/Adjust angle", m_angle);
    adjustAngle(m_angle);

    new Thread(
      () -> {
        try 
        {
          Thread.sleep(1000);
          resetHeading();
        } 
        catch (Exception e) 
        {
        // System.out.println("ERROR in sleep thread: " + e);
        }
      }).start();
    }

  public void stop() 
  {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  

  @Override
  public void periodic() 
  {

    m_odo.update(getAngle(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      });

    //SmartDashboard.putNumber("Angle", getAngle().getDegrees());
    
    m_totalCurrent = m_frontLeft.getDriveCurrent() + m_frontLeft.getTurnCurrent() + m_frontRight.getDriveCurrent() + m_frontRight.getTurnCurrent() + m_backLeft.getDriveCurrent() + m_backLeft.getTurnCurrent() + m_backRight.getDriveCurrent() + m_backRight.getTurnCurrent();
    //SmartDashboard.putNumber("Total Current", m_totalCurrent);

    m_field.setRobotPose(m_odo.getPoseMeters());
    SmartDashboard.putData("Swerve/Odo/Field", m_field);
    
    m_xStartPose = SmartDashboard.getNumber("Swerve/Odo/X", 2);
    m_yStartPose = SmartDashboard.getNumber("Swerve/Odo/Y", 2);
    SmartDashboard.putNumber("Swerve/Odo/X", m_xStartPose);
    SmartDashboard.putNumber("Swerve/Odo/Y", m_yStartPose);

    SmartDashboard.putNumber("Swerve/Odo/FrontLeftRotation", getFrontLeftRotation());

    SmartDashboard.putNumber("X", getPoseMeters().getX());
    SmartDashboard.putNumber("Y", getPoseMeters().getY());
    // SmartDashboard.putNumber("Swerve/Odo/FrontRightRotation", getFrontRightRotation());
    // SmartDashboard.putNumber("Swerve/Odo/BackLeftRotation", getBackLeftRotation());
    // SmartDashboard.putNumber("Swerve/Odo/BackRightRotation", getBackRightRotation());

    
    // // System.out.println("Chassis speeds:" + this.getChassisSpeeds());
    // // System.out.println("X error: " + (3 - m_odo.getPoseMeters().getX()));
  }
}
