// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.fasterxml.jackson.databind.util.Named;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.reduxrobotics.canand.CanandEventLoop;

import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.jar.Attributes.Name;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
//import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
//import com.pathplanner.lib.util.swerve.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.ArmToPos;
import frc.robot.commands.arm.SpinEndEffectorMotor;
import frc.robot.commands.elevator.ElevatorPosition;
import frc.robot.commands.elevator.ElevatorToPosition;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;

@SuppressWarnings("unused")

public class SwerveDrive extends SubsystemBase 
{
  private final SwerveModule m_frontLeft = new SwerveModule(
    DriveConstants.kFrontLeftDriveId, 
    DriveConstants.kFrontLeftTurnId,
    DriveConstants.kFrontLeftTurnEncoderPort, 
    DriveConstants.kFrontLeftTurnEncoderOffset, 
    DriveConstants.kFrontLeftDriveReversed,
    DriveConstants.kFrontLeftTurningReversed, 
    "FrontLeft"
  );

  private final SwerveModule m_frontRight = new SwerveModule(
    DriveConstants.kFrontRightDriveId, 
    DriveConstants.kFrontRightTurnId,
    DriveConstants.kFrontRightTurnEncoderPort, 
    DriveConstants.kFrontRightTurnEncoderOffset, 
    DriveConstants.kFrontRightDriveReversed,
    DriveConstants.kFrontRightTurningReversed, 
    "FrontRight"
  );

  private final SwerveModule m_backLeft = new SwerveModule(
    DriveConstants.kBackLeftDriveId, 
    DriveConstants.kBackLeftTurnId,
    DriveConstants.kBackLeftTurnEncoderPort, 
    DriveConstants.kBackLeftTurnEncoderOffset, 
    DriveConstants.kBackLeftDriveReversed, 
    DriveConstants.kBackLeftTurningReversed, 
    "BackLeft"
  );

  private final SwerveModule m_backRight = new SwerveModule(
    DriveConstants.kBackRightDriveId, 
    DriveConstants.kBackRightTurnId,
    DriveConstants.kBackRightTurnEncoderPort, 
    DriveConstants.kBackRightTurnEncoderOffset, 
    DriveConstants.kBackRightDriveReversed, 
    DriveConstants.kBackRightTurningReversed, 
    "BackRight"
  );

  int kModuleCount = 4;
  SwerveModule[] m_modules = {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
  public String[] m_moduleNames = {"frontLeft", "frontRight", "backLeft", "backRight"};
  int kUpdateFrequency = 50;
  protected final ReentrantReadWriteLock m_stateLock = new ReentrantReadWriteLock();
  //protected OdometryThread m_odometryThread;

  protected SwerveModulePosition[] m_modulePositions;
  protected SwerveModuleState[] m_moduleStates;

  private final Canandgyro m_imu = new Canandgyro(DriveConstants.kIMUCanID);

  private final SwerveDriveOdometry m_odo = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getAngle(), new SwerveModulePosition[] {
    m_frontLeft.getPosition(),
    m_frontRight.getPosition(),
    m_backLeft.getPosition(),
    m_backRight.getPosition()
  });


  


  private final Field2d m_field = new Field2d();



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
    double angle = m_imu.getYaw();
    angle *= 360;
    return Rotation2d.fromDegrees(angle);
  }

  

  public double getAngleDegrees()
  {
    return Math.IEEEremainder(-m_imu.getYaw() * 360 + 90, 360);
  }

  public ChassisSpeeds getChassisSpeeds() { 
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(), 
      m_frontRight.getState(), 
      m_backLeft.getState(), 
      m_backRight.getState()
    );
  }

  public SwerveModulePosition[] getModulePositions()
  {
    SwerveModulePosition[] positions = new SwerveModulePosition[kModuleCount];
    for (int i = 0; i < kModuleCount; i++)
    {
      positions[i] = m_modules[i].getPosition();
    }
    return positions;
  }

  public SwerveModuleState[] getModuleStates()
  {
    SwerveModuleState[] states = new SwerveModuleState[kModuleCount];
    for (int i = 0; i < kModuleCount; i++)
    {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  public void driveRobotRelative(ChassisSpeeds speeds){
    //ChassisSpeeds dis = ChassisSpeeds.discretize(speeds, 0.02); //needed to correct skew whilst rotating and translating simultaneously.
    SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(states);
  }

  public void resetOdoToPose(){
    m_odo.resetPosition(getAngle(), new SwerveModulePosition[] {
        m_frontLeft.getPosition(), m_frontRight.getPosition(),
        m_backLeft.getPosition(), m_backRight.getPosition()
      }, new Pose2d(m_xStartPose, m_yStartPose, getAngle()));
  }

  

  public SwerveDrive(Command elevatorToL2, Command elevatorToL3, Command elevatorToL4, Command armToL4) 
  {
    SmartDashboard.putData("Reset_Heading", resetHeadingCommand());
    CanandEventLoop.getInstance();
    // Leaving one here so I can remember how to do this later;
    // NamedCommands.registerCommand("Print", new PrintCommand("Print command is running!!!"));


    NamedCommands.registerCommand("Stow Elevator", m_elevatorToGround);
    NamedCommands.registerCommand("L4 Elevator", m_elevatorToL4);
    NamedCommands.registerCommand("Stow Arm", m_armToGroundIntake);
    NamedCommands.registerCommand("L4 Arm", m_armToL4);
    NamedCommands.registerCommand("Outtake End Effector", m_outtakeEndEffector);
    NamedCommands.registerCommand("Intake End Effector", m_intakeEndEffector);

    NamedCommands.registerCommand("Intake/Indexer Intake", m_spinIntakeIndexerRollers);
    NamedCommands.registerCommand("Intake/Indexer Outtake", m_outtakeIntakeIndexerRollers);


    // m_odometryThread = new OdometryThread();
    // m_odometryThread.start();

    try 
    {

      Translation2d[] t = {new Translation2d(0.368 - 0.0667, 0.368 - 0.0667),
        new Translation2d(0.368  - 0.0667, -0.368 + 0.0667),
        new Translation2d(-0.368 + 0.0667,  0.368 - 0.0667),
        new Translation2d(-0.368 + 0.0667, -0.368 + 0.0667)};
        
      //RobotConfig config = RobotConfig.fromGUISettings();
      // DCMotor m_krakenConfig = new DCMotor(12, 7.09, 366.0, 2.0, 628.0, 1);
      // ModuleConfig m_moduleConfig = new ModuleConfig(0.051, 5.0, 1.2, m_krakenConfig, 130, 1);
      //RobotConfig m_robotConfig = new RobotConfig(51, 4.6, m_moduleConfig, t);
      RobotConfig m_robotConfig = RobotConfig.fromGUISettings();
      
      AutoBuilder.configure
      (
        this::getPoseMeters,
        this::resetOdo,
        this::getChassisSpeeds, 
        this::driveRobotRelative, 
        new PPHolonomicDriveController(DriveConstants.kTranslationConstants, DriveConstants.kRotationConstants), 
        m_robotConfig,
        () -> 
        {
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

      System.out.println("configured auto");



      //SmartDashboard.putData("Swerve/Distance/reset", new InstantCommand(this::resetAllDistances));
      double m_angle = SmartDashboard.getNumber("Driving/Adjust angle", 0);
      //SmartDashboard.putNumber("Driving/Adjust angle", m_angle);
      adjustAngle(m_angle);

      new Thread(() -> {
          try 
          {
            Thread.sleep(1000);
            resetHeading();
          } 
          catch (Exception e) 
          {
            System.out.println("ERROR in sleep thread: " + e);
          }
        }).start();
    // }
    // catch(Exception e)
    // {
    //   System.out.println("RobotConfig GUI Settings error");
    // }
      }
  }
  
  
  public void stop() 
  {
    m_frontLeft.stop();
    m_frontRight.stop();
    m_backLeft.stop();
    m_backRight.stop();
  }

  // public class OdometryThread
  // {
  //   protected static final int kThreadPriority = 3;
  //   protected final Thread m_thread; 
  //   protected volatile boolean m_running = false;
    
  //   protected final BaseStatusSignal[] m_allSignals;

  //   protected int lastThreadPriority = kThreadPriority;
  //   protected volatile int threadPriorityToSet = kThreadPriority;

  //   public OdometryThread()
  //   {
  //     m_thread = new Thread(this::run);
  //     m_thread.setDaemon(true);

  //     //drivePos, driveVel, 
  //     m_allSignals = new BaseStatusSignal[(kModuleCount * SwerveModule.m_numberOfStatusSignals)];

      
  //     for (int i = 0; i < kModuleCount; ++i) 
  //     {
  //         m_allSignals[(i * SwerveModule.m_numberOfStatusSignals) + 0] = m_modules[i].getDrivePosStatusSignal();
  //         m_allSignals[(i * SwerveModule.m_numberOfStatusSignals) + 1] = m_modules[i].getDriveVelStatusSignal();
  //     }
  //   }
    
  //   public void start() 
  //   {
  //     m_running = true;
  //     m_thread.start();
  //   }

  //   public void stop(long millis)
  //   {
  //     m_running = false;
  //     try
  //     {
  //       m_thread.join(millis);
  //     }
  //     catch (final InterruptedException e)
  //     {
  //       Thread.currentThread().interrupt();
  //     }
  //   }

  //   public void run()
  //   {
  //     BaseStatusSignal.setUpdateFrequencyForAll(kUpdateFrequency, m_allSignals);
  //     Threads.setCurrentThreadPriority(true, kThreadPriority);

  //     while(m_running)
  //     {
  //       try{
  //         Thread.sleep(1/kUpdateFrequency);
  //       }
  //       catch(Exception e)
  //       {
  //         ;
  //       }
  //       try
  //       {
  //         m_stateLock.writeLock().lock();

  //         // m_modulePositions = getModulePositions();
  //         // m_moduleStates = getModuleStates();
          
  //         // m_odo.update(getAngle(), m_modulePositions);

  //       }
  //       finally
  //       {
  //         m_stateLock.writeLock().unlock();
  //       }
  //     }
  //   }
  // }

  

  @Override
  public void periodic() 
  {
    m_modulePositions = getModulePositions();
    m_moduleStates = getModuleStates();
    m_odo.update(getAngle(), m_modulePositions);
    SmartDashboard.putNumber("X", m_odo.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odo.getPoseMeters().getY());

    m_field.setRobotPose(m_odo.getPoseMeters());
    // SmartDashboard.putData("Swerve/Odo/Field", m_field);

    // SmartDashboard.putNumber("X", getPoseMeters().getX());
    // SmartDashboard.putNumber("Y", getPoseMeters().getY());
    
    // SmartDashboard.putData("Reset_Heading", resetHeadingCommand());
  }
}
