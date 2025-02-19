// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.DriftSwerveDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class DriftSwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */

  private final Joystick m_joystick;
  private final SlewRateLimiter m_xSlewRateLimiter;
  private final SlewRateLimiter m_ySlewRateLimiter;
  private final DriftSwerveDrive m_swerveDrive;
  private final Vision m_vision;

  private double m_xSpeed, m_ySpeed, m_turningSpeed, m_xySpeed;

  private final JoystickButton m_turnTo1stTag, m_turnTo2ndTag, m_turnTo3rdTag, m_turnTo4thTag, m_turnTo5thTag, m_turnTo6thTag, m_centerToTag;
  private PIDController m_pid = new PIDController(DriveConstants.kTurnToAngleP, DriveConstants.kTurnToAngleI, DriveConstants.kTurnToAngleD);
  private double m_targetAngle;
  private double m_currentYaw;
  private double m_currentAngle;
  private int m_targetID;
  private final Optional<Alliance> m_alliance = DriverStation.getAlliance(); 

  public DriftSwerveJoystick(DriftSwerveDrive swerveDrive, Joystick joystick, Vision vision, JoystickButton turnTo1stTag, JoystickButton turnTo2ndTag, JoystickButton turnTo3rdTag, JoystickButton turnTo4thTag, JoystickButton turnTo5thTag, JoystickButton turnTo6thTag, JoystickButton centerToTag) {
    m_xSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_ySlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_joystick = joystick;
    m_swerveDrive = swerveDrive;
    m_vision = vision;

    m_turnTo1stTag = turnTo1stTag;
    m_turnTo2ndTag = turnTo2ndTag;
    m_turnTo3rdTag = turnTo3rdTag;
    m_turnTo4thTag = turnTo4thTag;
    m_turnTo5thTag = turnTo5thTag;
    m_turnTo6thTag = turnTo6thTag;
    m_centerToTag = centerToTag;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("SwerveJoystick Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_xSpeed = m_joystick.getRawAxis(IOConstants.kJoystickXAxis);
    m_ySpeed = m_joystick.getRawAxis(IOConstants.kJoystickYAxis);
    m_turningSpeed = m_joystick.getRawAxis(IOConstants.kJoystickRotAxis);
    //Makes the speed response exponential in relation to the joystick input.
    //That way, the first little bit of joystick input gives more control.  
    m_xSpeed = Math.signum(m_xSpeed) * (Math.pow(2, Math.abs(m_xSpeed)) -1) * -1;
    m_ySpeed = Math.signum(m_ySpeed) * (Math.pow(2, Math.abs(m_ySpeed)) -1) * -1;
    m_turningSpeed = Math.signum(m_turningSpeed) * (Math.pow(2, Math.abs(m_turningSpeed)) -1) * -1;
    
    // SmartDashboard.putNumber("Joystick/xSpeedRaw", xSpeed);
    // SmartDashboard.putNumber("Joystick/ySpeedRaw", ySpeed);
    // SmartDashboard.putNumber("Joystick/turningSpeedRaw", turningSpeed);

    //apply deadband
    m_xSpeed = Math.abs(m_xSpeed) > IOConstants.kDeadband ? m_xSpeed : 0.0;
    m_ySpeed = Math.abs(m_ySpeed) > IOConstants.kDeadband ? m_ySpeed : 0.0;
    m_turningSpeed = Math.abs(m_turningSpeed) > IOConstants.kDeadband ? m_turningSpeed : 0.0;

    //use SlewRateLimiter with DriveConstants
    m_xSpeed = m_xSlewRateLimiter.calculate(m_xSpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    m_ySpeed = m_ySlewRateLimiter.calculate(m_ySpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    m_turningSpeed = m_turningSpeed * DriveConstants.kMaxTurningRadiansPerSecond;
    // SmartDashboard.putNumber("Joystick/xSpeedCommanded", xSpeed);
    // SmartDashboard.putNumber("Joystick/ySpeedCommanded", ySpeed);
    // SmartDashboard.putNumber("Joystick/turningSpeedCommanded", turningSpeed);
    //Logger.recordOutput(getName(), desiredSwerveModuleStates);

    m_currentAngle = m_swerveDrive.getAngleDegrees();
    SmartDashboard.putNumber("Current angle", m_currentAngle);
    if (m_turnTo1stTag.getAsBoolean()){
      m_targetAngle = VisionConstants.kTag10Angle;
      m_targetID = 10;
      m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
    }
    if (m_turnTo1stTag.getAsBoolean()){
      if (m_alliance.isPresent()) {
        System.out.println("True");
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag10Angle;
          m_targetID = 10;
        } else if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag21Angle;
          m_targetID = 21;
        }
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_turnTo2ndTag.getAsBoolean()) {
      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag9Angle;
          m_targetID = 9;
        } else if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag22Angle;
          m_targetID = 22;
        }
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_turnTo3rdTag.getAsBoolean()) {
      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag8Angle;
          m_targetID = 8;
        } else if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag17Angle;
          m_targetID = 17;
        }
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_turnTo4thTag.getAsBoolean()) {
      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag7Angle;  
          m_targetID = 7;
        } else 
        if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag18Angle;  
          m_targetID = 18;
        } 
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_turnTo5thTag.getAsBoolean()) {
      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag6Angle;
          m_targetID = 6;
        } else 
        if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag19Angle;
          m_targetID = 19;
        } 
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_turnTo6thTag.getAsBoolean()) {
      if (m_alliance.isPresent()) {
        if (m_alliance.get() == Alliance.Red) {
          m_targetAngle = VisionConstants.kTag11Angle;
          m_targetID = 11;
        } else if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle = VisionConstants.kTag20Angle;
          m_targetID = 20;
        } 
        m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
      }
    } else if (m_centerToTag.getAsBoolean()) {
      m_currentYaw = m_vision.getTargetYaw();
      m_xySpeed = m_pid.calculate(m_currentYaw, 0);
      m_xSpeed = m_xySpeed*Math.cos(Math.toRadians(VisionConstants.kTagAngles[m_targetID - 6]));
      m_ySpeed = m_xySpeed*Math.sin(Math.toRadians(VisionConstants.kTagAngles[m_targetID - 6]));
    } 

    m_vision.setTargetID(m_targetID);
        
    if (m_turningSpeed > DriveConstants.kMaxTurningRadiansPerSecond){
      m_turningSpeed = DriveConstants.kMaxTurningRadiansPerSecond;
    }

    SmartDashboard.putNumber("X speed", m_xSpeed);
    SmartDashboard.putNumber("Y speed", m_ySpeed);
    SmartDashboard.putNumber("Turning speed", m_turningSpeed);
    System.out.println("Turning to " + m_targetID + "Degrees: " + m_targetAngle);
    System.out.println("Turning speed (-1 to 1): " + m_turningSpeed);

    m_swerveDrive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed, m_swerveDrive.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_xSpeed = 0;
    m_ySpeed = 0;
    m_turningSpeed = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
