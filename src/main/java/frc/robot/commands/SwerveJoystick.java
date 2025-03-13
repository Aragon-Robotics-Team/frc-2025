// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class SwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */

  private final Joystick m_joystick;
  private final SlewRateLimiter m_xSlewRateLimiter;
  private final SlewRateLimiter m_ySlewRateLimiter;
  private final SwerveDrive m_swerveDrive;
  public int m_driveMode = 0;
  private final Vision m_vision;

  private double m_xSpeed, m_ySpeed, m_turningSpeed, m_xySpeed;

  // private final JoystickButton m_turnTo1stTag, m_turnTo2ndTag, m_turnTo3rdTag, m_turnTo4thTag, m_turnTo5thTag, m_turnTo6thTag, m_centerToTag;
  // private final JoystickButton m_selectBestTag, m_centerToLeftPole, m_centerToRightPole;
  // private final JoystickButton m_turnToTag6Left, m_turnToTag6Right, m_turnToTag7Left, m_turnToTag7Right, m_turnToTag8Left, m_turnToTag8Right, m_turnToTag9Left, m_turnToTag9Right, m_turnToTag10Left, m_turnToTag10Right, m_turnToTag11Left, m_turnToTag11Right, m_turnToTag17Left, m_turnToTag17Right, m_turnToTag18Left, m_turnToTag18Right, m_turnToTag19Left, m_turnToTag19Right, m_turnToTag20Left, m_turnToTag20Right, m_turnToTag21Left, m_turnToTag21Right, m_turnToTag22Left, m_turnToTag22Right;
  private PIDController m_turningPID = new PIDController(DriveConstants.kTurnToAngleP, DriveConstants.kTurnToAngleI, DriveConstants.kTurnToAngleD);
  private PIDController m_xPID = new PIDController(DriveConstants.kDriveToXP, DriveConstants.kDriveToXI, DriveConstants.kDriveToXD);
  private PIDController m_yPID = new PIDController(DriveConstants.kDriveToYP, DriveConstants.kDriveToYI, DriveConstants.kDriveToYD);
  private double m_targetAngle, m_targetX, m_targetY;
  private double m_currentYaw;
  private double m_currentAngle;
  private int m_targetID = 0;
  private final Optional<Alliance> m_alliance = DriverStation.getAlliance();
  // // Tag ID --> Left/ Right --> Pose 2d with x, y, rotation
  private HashMap<Integer, HashMap<String, Pose2d>> m_polePoses = new HashMap<Integer, HashMap<String, Pose2d>>();
  // Button ID --> Red/ Blue --> Left/ Right --> Pose 2d with x, y, rotation
  // private HashMap<Integer, HashMap<String, HashMap<String, Pose2d>>> m_polePoses = new HashMap<Integer, HashMap<String, HashMap<String, Pose2d>>>();
  // public SwerveJoystick(SwerveDrive swerveDrive, Joystick joystick, Vision vision, JoystickButton turnTo1stTag, JoystickButton turnTo2ndTag, JoystickButton turnTo3rdTag, JoystickButton turnTo4thTag, JoystickButton turnTo5thTag, JoystickButton turnTo6thTag, JoystickButton centerToTag) {
  // public SwerveJoystick(SwerveDrive swerveDrive, Joystick joystick, Vision vision, JoystickButton selectBestTag, JoystickButton centerToLeftPole, JoystickButton centerToRightPole) {
  public SwerveJoystick(SwerveDrive swerveDrive, Joystick joystick, Vision vision) {
    m_xSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_ySlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_joystick = joystick;
    m_swerveDrive = swerveDrive;
    m_vision = vision;

    // m_selectBestTag = selectBestTag;
    // m_centerToLeftPole = centerToLeftPole;
    // m_centerToRightPole = centerToRightPole;

    m_polePoses.put(6, new HashMap<String, Pose2d>());
    m_polePoses.get(6).put("Left", VisionConstants.kTag6Left);
    m_polePoses.get(6).put("Right", VisionConstants.kTag6Right);
    
    m_polePoses.put(7, new HashMap<String, Pose2d>());
    m_polePoses.get(7).put("Left", VisionConstants.kTag7Left);
    m_polePoses.get(7).put("Right", VisionConstants.kTag7Right);
    
    m_polePoses.put(8, new HashMap<String, Pose2d>());
    m_polePoses.get(8).put("Left", VisionConstants.kTag8Left);
    m_polePoses.get(8).put("Right", VisionConstants.kTag8Right);

    m_polePoses.put(9, new HashMap<String, Pose2d>());
    m_polePoses.get(9).put("Left", VisionConstants.kTag9Left);
    m_polePoses.get(9).put("Right", VisionConstants.kTag9Right);

    m_polePoses.put(10, new HashMap<String, Pose2d>());
    m_polePoses.get(10).put("Left", VisionConstants.kTag10Left);
    m_polePoses.get(10).put("Right", VisionConstants.kTag10Right);

    m_polePoses.put(11, new HashMap<String, Pose2d>());
    m_polePoses.get(11).put("Left", VisionConstants.kTag11Left);
    m_polePoses.get(11).put("Right", VisionConstants.kTag11Right);

    m_polePoses.put(17, new HashMap<String, Pose2d>());
    m_polePoses.get(17).put("Left", VisionConstants.kTag17Left);
    m_polePoses.get(17).put("Right", VisionConstants.kTag17Right);

    m_polePoses.put(18, new HashMap<String, Pose2d>());
    m_polePoses.get(18).put("Left", VisionConstants.kTag18Left);
    m_polePoses.get(18).put("Right", VisionConstants.kTag18Right);

    m_polePoses.put(19, new HashMap<String, Pose2d>());
    m_polePoses.get(19).put("Left", VisionConstants.kTag19Left);
    m_polePoses.get(19).put("Right", VisionConstants.kTag19Right);

    m_polePoses.put(20, new HashMap<String, Pose2d>());
    m_polePoses.get(20).put("Left", VisionConstants.kTag20Left);
    m_polePoses.get(20).put("Right", VisionConstants.kTag20Right);

    m_polePoses.put(21, new HashMap<String, Pose2d>());
    m_polePoses.get(21).put("Left", VisionConstants.kTag21Left);
    m_polePoses.get(21).put("Right", VisionConstants.kTag21Right);

    m_polePoses.put(22, new HashMap<String, Pose2d>());
    m_polePoses.get(22).put("Left", VisionConstants.kTag22Left);
    m_polePoses.get(22).put("Right", VisionConstants.kTag22Right);

    

    // m_turnTo1stTag = turnTo1stTag;
    // m_turnTo2ndTag = turnTo2ndTag;
    // m_turnTo3rdTag = turnTo3rdTag;
    // m_turnTo4thTag = turnTo4thTag;
    // m_turnTo5thTag = turnTo5thTag;
    // m_turnTo6thTag = turnTo6thTag;
    // m_centerToTag = centerToTag;

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
    double xSpeed = -m_joystick.getRawAxis(IOConstants.kJoystickYAxis);
    double ySpeed = -m_joystick.getRawAxis(IOConstants.kJoystickXAxis);
    double turningSpeed = -m_joystick.getRawAxis(IOConstants.kJoystickRotAxis);


    // if (m_driveMode == 0) { //standard mode;
    // //Makes the speed response exponential in relation to the joystick input.
    // //That way, the first little bit of joystick input gives more control.  
    // xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -1;
    // ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -1;
    // turningSpeed = Math.signum(turningSpeed) * (Math.pow(2, Math.abs(turningSpeed)) -1) * -1;
    // //xSpeed = Math.pow(xSpeed, 5);
    // // ySpeed = Math.pow(ySpeed, 5);
    // // turningSpeed = Math.pow(turningSpeed, 5);
    // }
    // else if(m_driveMode == 1)
    // {
    //   xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -0.1;
    //   ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -0.1;
    //   turningSpeed = Math.signum(turningSpeed) * (Math.pow(2, Math.abs(turningSpeed)) -1) * -0.1;
    // }
  
      //xSpeed = Math.pow(xSpeed, 5);
      // ySpeed = Math.pow(ySpeed, 5);
      // turningSpeed = Math.pow(turningSpeed, 5);
    
    System.out.println("Before appliance " + m_turningSpeed);
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
    SmartDashboard.putNumber("Joystick/xSpeedCommanded", m_xSpeed);
    SmartDashboard.putNumber("Joystick/ySpeedCommanded", m_ySpeed);
    SmartDashboard.putNumber("Joystick/turningSpeedCommanded", m_turningSpeed);
    //Logger.recordOutput(getName(), desiredSwerveModuleStates);

    
    switch (DriverStation.getStickButtons(2)) {
      case 1: // Button A on joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 10;
            break;
          case Blue:
            m_targetID = 21;
            break;
        }
        break;
      case 2: // Button B on joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 9;
            break;
          case Blue:
            m_targetID = 22;
            break;
        }
        break;
      case 4: // Button X on Joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 8;
            break;
          case Blue:
            m_targetID = 17;
            break;
        }
        break;
      case 8: // Button Y on Joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 7;
            break;
          case Blue:
            m_targetID = 18;
            break;
        }
        break;
      case 16: // Left bumper on Joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 6;
            break;
          case Blue:
            m_targetID = 19;
            break;
        }
        break;
      case 32: // Right bumper on Joystick
        switch (m_alliance.get()) {
          case Red:
            m_targetID = 11;
          case Blue:
            m_targetID = 20;
        }
        break;
    }

    m_currentAngle = m_swerveDrive.getAngle().getDegrees();
    SmartDashboard.putNumber("Current angle", m_currentAngle);
    System.out.println("Target ID: " + m_targetID);
    System.out.println("Constants tag 8 angle: " + VisionConstants.kTag8Left.getRotation().getDegrees());
    if (m_targetID != 0) {
      if (m_joystick.getRawButton(IOConstants.kVisionSnapToAngleButtonID)) {
        m_targetAngle = m_polePoses.get(m_targetID).get("Left").getRotation().getDegrees();
        System.out.println("Target angle: " + m_targetAngle);

        m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
      } else if (m_joystick.getRawButton(IOConstants.kVisionLeftAlignButtonID)) {
        m_targetX = m_polePoses.get(m_targetID).get("Left").getX();
        m_targetY = m_polePoses.get(m_targetID).get("Left").getY();

        m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
        m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
        m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);
      } else if (m_joystick.getRawButton(IOConstants.kVisionRightAlignButtonID)) {
        m_targetX = m_polePoses.get(m_targetID).get("Right").getX();
        m_targetY = m_polePoses.get(m_targetID).get("Right").getY();

        m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
        m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
        m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);
      }
    }
    // if (m_selectBestTag.getAsBoolean()) {
    //   // if (m_vision.hasTargets()) { 
    //     m_targetID = m_vision.getID();
    //     m_targetAngle = m_polePoses.get(m_targetID).get("Left").getRotation().getDegrees();

    //     m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
    //   // }
    // } else if (m_centerToLeftPole.getAsBoolean()) { 
    //   // if (m_vision.hasTargets()) {     
    //     m_targetX = m_polePoses.get(m_targetID).get("Left").getX();
    //     m_targetY = m_polePoses.get(m_targetID).get("Left").getY();

    //     m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
    //     m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
    //     m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);
    //   // }
    // } else if (m_centerToRightPole.getAsBoolean()) {
    //   // if (m_vision.hasTargets()) {
    //     m_targetX = m_polePoses.get(m_targetID).get("Right").getX();
    //     m_targetY = m_polePoses.get(m_targetID).get("Right").getY();

    //     m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
    //     m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
    //     m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);
    //   // }
    // }

    m_vision.setTargetID(m_targetID);
        
    if (m_turningSpeed > DriveConstants.kMaxTurningRadiansPerSecond){
      m_turningSpeed = DriveConstants.kMaxTurningRadiansPerSecond;
    } else if (m_xSpeed > DriveConstants.kMaxTranslationalMetersPerSecond) {
      m_xSpeed = DriveConstants.kMaxTranslationalMetersPerSecond;
    } else if (m_ySpeed > DriveConstants.kMaxTranslationalMetersPerSecond) {
      m_ySpeed = DriveConstants.kMaxTranslationalMetersPerSecond;
    }

    SmartDashboard.putNumber("X speed", m_xSpeed);
    SmartDashboard.putNumber("Y speed", m_ySpeed);
    SmartDashboard.putNumber("Turning speed", m_turningSpeed);
    System.out.println("Turning to " + m_targetID + " Degrees: " + m_targetAngle);

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
