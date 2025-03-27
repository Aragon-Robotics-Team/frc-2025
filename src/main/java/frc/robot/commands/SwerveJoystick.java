// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  private Constraints m_trapezoidalConstraints = new Constraints(DriveConstants.kMaxTranslationalMetersPerSecond, DriveConstants.kMaxTurningRadiansPerSecond);
  // private PIDController m_turningPID = new PIDController(DriveConstants.kTurnToAngleP, DriveConstants.kTurnToAngleI, DriveConstants.kTurnToAngleD);
  // private PIDController m_xPID = new PIDController(DriveConstants.kDriveToXP, DriveConstants.kDriveToXI, DriveConstants.kDriveToXD);
  // private PIDController m_yPID = new PIDController(DriveConstants.kDriveToYP, DriveConstants.kDriveToYI, DriveConstants.kDriveToYD);
  // private ProfiledPIDController m_turningPID = new ProfiledPIDController(DriveConstants.kTurnToAngleP, DriveConstants.kTurnToAngleI, DriveConstants.kTurnToAngleD, m_trapezoidalConstraints);
  // private ProfiledPIDController m_xPID = new ProfiledPIDController(DriveConstants.kDriveToXP, DriveConstants.kDriveToXI, DriveConstants.kDriveToXD, m_trapezoidalConstraints);
  // private ProfiledPIDController m_yPID = new ProfiledPIDController(DriveConstants.kDriveToYP, DriveConstants.kDriveToYI, DriveConstants.kDriveToYD, m_trapezoidalConstraints);
  private double m_xP = 0.1;
  private double m_xI = 0;
  private double m_xD = 0;

  private double m_yP = 0.2;
  private double m_yI = 0;
  private double m_yD = 0;

  private double m_turningP = 0.1;
  private double m_turningI = 0;
  private double m_turningD = 0;
  
  // private ProfiledPIDController m_xPID = new ProfiledPIDController(m_xP, m_xI, m_xD, m_trapezoidalConstraints);
  // private ProfiledPIDController m_yPID = new ProfiledPIDController(m_yP, m_yI, m_yD, m_trapezoidalConstraints);
  // private ProfiledPIDController m_turningPID = new ProfiledPIDController(m_turningP, m_turningI, m_turningD, m_trapezoidalConstraints);
  
  private PIDController m_xPID = new PIDController(m_xP, m_xI, m_xD);
  private PIDController m_yPID = new PIDController(m_yP, m_yI, m_yD);
  private PIDController m_turningPID = new PIDController(m_turningP, m_turningI, m_turningD);

  private double m_targetAngle, m_targetX, m_targetY;
  private Pose2d m_targetPose;
  private double m_currentYaw;
  private double m_currentAngle;
  private int m_targetID = 0;
  private final Optional<Alliance> m_alliance = DriverStation.getAlliance();
  private final AprilTagFieldLayout m_fieldLayout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
  private List<Pose2d> m_tagPoses = new ArrayList<Pose2d>();
  private List<Integer> m_frontTags = new ArrayList<Integer>();
  private List<Integer> m_backTags = new ArrayList<Integer>();
  // // Tag ID --> Left/ Right --> Pose 2d with x, y, rotation
  // private HashMap<Integer, HashMap<String, Pose2d>> m_polePoses = new HashMap<Integer, HashMap<String, Pose2d>>();
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

    for (int tag : VisionConstants.kTagIDs){
      m_tagPoses.add(m_fieldLayout.getTagPose(tag).get().toPose2d());
    }

    for (int tagID : VisionConstants.kFrontTagIDs) {
      m_frontTags.add(tagID);
    }

    for (int tagID : VisionConstants.kBackTagIDs) {
      m_backTags.add(tagID);
    }

    // m_selectBestTag = selectBestTag;
    // m_centerToLeftPole = centerToLeftPole;
    // m_centerToRightPole = centerToRightPole;

    // m_polePoses.put(6, new HashMap<String, Pose2d>());
    // m_polePoses.get(6).put("Left", VisionConstants.kTag6Left);
    // m_polePoses.get(6).put("Right", VisionConstants.kTag6Right);
    
    // m_polePoses.put(7, new HashMap<String, Pose2d>());
    // m_polePoses.get(7).put("Left", VisionConstants.kTag7Left);
    // m_polePoses.get(7).put("Right", VisionConstants.kTag7Right);
    
    // m_polePoses.put(8, new HashMap<String, Pose2d>());
    // m_polePoses.get(8).put("Left", VisionConstants.kTag8Left);
    // m_polePoses.get(8).put("Right", VisionConstants.kTag8Right);

    // m_polePoses.put(9, new HashMap<String, Pose2d>());
    // m_polePoses.get(9).put("Left", VisionConstants.kTag9Left);
    // m_polePoses.get(9).put("Right", VisionConstants.kTag9Right);

    // m_polePoses.put(10, new HashMap<String, Pose2d>());
    // m_polePoses.get(10).put("Left", VisionConstants.kTag10Left);
    // m_polePoses.get(10).put("Right", VisionConstants.kTag10Right);

    // m_polePoses.put(11, new HashMap<String, Pose2d>());
    // m_polePoses.get(11).put("Left", VisionConstants.kTag11Left);
    // m_polePoses.get(11).put("Right", VisionConstants.kTag11Right);

    // m_polePoses.put(17, new HashMap<String, Pose2d>());
    // m_polePoses.get(17).put("Left", VisionConstants.kTag17Left);
    // m_polePoses.get(17).put("Right", VisionConstants.kTag17Right);

    // m_polePoses.put(18, new HashMap<String, Pose2d>());
    // m_polePoses.get(18).put("Left", VisionConstants.kTag18Left);
    // m_polePoses.get(18).put("Right", VisionConstants.kTag18Right);

    // m_polePoses.put(19, new HashMap<String, Pose2d>());
    // m_polePoses.get(19).put("Left", VisionConstants.kTag19Left);
    // m_polePoses.get(19).put("Right", VisionConstants.kTag19Right);

    // m_polePoses.put(20, new HashMap<String, Pose2d>());
    // m_polePoses.get(20).put("Left", VisionConstants.kTag20Left);
    // m_polePoses.get(20).put("Right", VisionConstants.kTag20Right);

    // m_polePoses.put(21, new HashMap<String, Pose2d>());
    // m_polePoses.get(21).put("Left", VisionConstants.kTag21Left);
    // m_polePoses.get(21).put("Right", VisionConstants.kTag21Right);

    // m_polePoses.put(22, new HashMap<String, Pose2d>());
    // m_polePoses.get(22).put("Left", VisionConstants.kTag22Left);
    // m_polePoses.get(22).put("Right", VisionConstants.kTag22Right);

    

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = -m_joystick.getRawAxis(IOConstants.kJoystickYAxis);
    double ySpeed = -m_joystick.getRawAxis(IOConstants.kJoystickXAxis);
    double turningSpeed = -m_joystick.getRawAxis(IOConstants.kJoystickRotAxis);

    SmartDashboard.putData("X PID", m_xPID);
    SmartDashboard.putData("Y PID", m_yPID);
    SmartDashboard.putData("Turning PID", m_turningPID);

    SmartDashboard.putNumber("XP", m_xP);
    // SmartDashboard.putNumber("XI", m_xI);
    // SmartDashboard.putNumber("XD", m_xD);
    SmartDashboard.putNumber("yP", m_yP);
    // SmartDashboard.putNumber("yI", m_yI);
    // SmartDashboard.putNumber("yD", m_yD);
    SmartDashboard.putNumber("turningP", m_turningP);
    // SmartDashboard.putNumber("turningI", m_turningI);  
    SmartDashboard.putNumber("Turning Speed", m_turningSpeed);
    SmartDashboard.putNumber("X speed", m_xSpeed);
    SmartDashboard.putNumber("Y speed", m_ySpeed);
    SmartDashboard.putNumber("Target X", m_targetX);
    SmartDashboard.putNumber("Target Y", m_targetY);
    SmartDashboard.putNumber("Target Angle", m_targetAngle);
    SmartDashboard.putNumber("Target ID", m_targetID);

    // m_xPID.setP(m_xP);
    // m_xPID.setI(m_xI);
    // m_xPID.setD(m_xD);
    // m_yPID.setP(m_yP);
    // m_yPID.setI(m_yI);
    // m_yPID.setD(m_yD);
    // m_turningPID.setP(0.7);
    // m_turningPID.setI(0);
    // m_turningPID.setD(0);



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
    //Logger.recordOutput(getName(), desiredSwerveModuleStates);

    
    // switch (DriverStation.getStickButtons(2)) {
    //   case 1: // Button A on joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 10;
    //         break;
    //       case Blue:
    //         m_targetID = 21;
    //         break;
    //     }
    //     break;
    //   case 2: // Button B on joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 9;
    //         break;
    //       case Blue:
    //         m_targetID = 22;
    //         break;
    //     }
    //     break;
    //   case 4: // Button X on Joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 8;
    //         break;
    //       case Blue:
    //         m_targetID = 17;
    //         break;
    //     }
    //     break;
    //   case 8: // Button Y on Joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 7;
    //         break;
    //       case Blue:
    //         m_targetID = 18;
    //         break;
    //     }
    //     break;
    //   case 16: // Left bumper on Joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 6;
    //         break;
    //       case Blue:
    //         m_targetID = 19;
    //         break;
    //     }
    //     break;
    //   case 32: // Right bumper on Joystick
    //     switch (m_alliance.get()) {
    //       case Red:
    //         m_targetID = 11;
    //       case Blue:
    //         m_targetID = 20;
    //     }
    //     break;
    // }

    m_currentAngle = m_swerveDrive.getAngle().getDegrees();
    
      if (m_joystick.getRawButton(IOConstants.kVisionSnapToAngleButtonID)) {
        m_targetPose = m_swerveDrive.getEstimatedPosition().nearest(m_tagPoses);
        m_targetID = VisionConstants.kTagIDs[m_tagPoses.indexOf(m_targetPose)];
        m_targetAngle = m_targetPose.getRotation().getDegrees() - 180;
        if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle *= -1;
        }

        // m_targetAngle = m_polePoses.get(m_targetID).get("Left").getRotation().getDegrees();

        // m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
        m_turningSpeed = m_turningPID.calculate(m_currentAngle, 120); // Just for PID tuning!!!
        
      } else if (m_joystick.getRawButton(IOConstants.kVisionLeftAlignButtonID)) {
        m_targetPose = m_swerveDrive.getEstimatedPosition().nearest(m_tagPoses);
        m_targetID = VisionConstants.kTagIDs[m_tagPoses.indexOf(m_targetPose)];
        m_targetAngle = m_targetPose.getRotation().getDegrees() - 180;
        if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle *= -1;
        }

        // m_targetX = m_polePoses.get(m_targetID).get("Left").getX();
        // m_targetY = m_polePoses.get(m_targetID).get("Left").getY();

        // m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);
        if (m_frontTags.contains(m_targetID)) {
          m_targetX = m_fieldLayout.getTagPose(m_targetID).get().getX() + VisionConstants.kPoleDistance*Math.cos(m_targetAngle - 90);
          m_targetY = m_fieldLayout.getTagPose(m_targetID).get().getY() + VisionConstants.kPoleDistance*Math.sin(m_targetAngle - 90);
        } else if (m_backTags.contains(m_targetID)) {
          m_targetX = m_fieldLayout.getTagPose(m_targetID).get().getX() - VisionConstants.kPoleDistance*Math.cos(m_targetAngle - 90);
          m_targetY = m_fieldLayout.getTagPose(m_targetID).get().getY() - VisionConstants.kPoleDistance*Math.sin(m_targetAngle - 90);
        }

        // m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
        // m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);

        // Just for PID tuning purposes
        m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), 2);
        m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), 2);
      } else if (m_joystick.getRawButton(IOConstants.kVisionRightAlignButtonID)) {
        m_targetPose = m_swerveDrive.getEstimatedPosition().nearest(m_tagPoses);
        m_targetID = VisionConstants.kTagIDs[m_tagPoses.indexOf(m_targetPose)];
        m_targetAngle = m_targetPose.getRotation().getDegrees() - 180;
        if (m_alliance.get() == Alliance.Blue) {
          m_targetAngle *= -1;
        }
        // m_targetX = m_polePoses.get(m_targetID).get("Right").getX();
        // m_targetY = m_polePoses.get(m_targetID).get("Right").getY();

        // m_turningSpeed = m_turningPID.calculate(m_currentAngle, m_targetAngle);

        if (m_frontTags.contains(m_targetID)) {
          m_targetX = m_fieldLayout.getTagPose(m_targetID).get().getX() - VisionConstants.kPoleDistance*Math.cos(m_targetAngle - 90);
          m_targetY = m_fieldLayout.getTagPose(m_targetID).get().getY() - VisionConstants.kPoleDistance*Math.sin(m_targetAngle - 90);
        } else if (m_backTags.contains(m_targetID)) {
          m_targetX = m_fieldLayout.getTagPose(m_targetID).get().getX() + VisionConstants.kPoleDistance*Math.cos(m_targetAngle - 90);
          m_targetY = m_fieldLayout.getTagPose(m_targetID).get().getY() + VisionConstants.kPoleDistance*Math.sin(m_targetAngle - 90);
        }

        // m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), m_targetX);
        // m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), m_targetY);

        // Just for PID tuning purposes
        m_xSpeed = m_xPID.calculate(m_swerveDrive.getEstimatedPosition().getX(), 2);
        m_ySpeed = m_yPID.calculate(m_swerveDrive.getEstimatedPosition().getY(), 2);
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

  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("X speed", () -> m_xSpeed, null);
    builder.addDoubleProperty("Y speed", () -> m_ySpeed, null);
    builder.addDoubleProperty("Turning speed", () -> m_turningSpeed, null);
    builder.addDoubleProperty("X Pose", () -> m_swerveDrive.getEstimatedPosition().getX(), null);
    builder.addDoubleProperty("Y pose", () -> m_swerveDrive.getEstimatedPosition().getY(), null);
    builder.addDoubleProperty("Target ID", () -> m_targetID, null);
    builder.addDoubleProperty("Target Angle", () -> m_targetAngle, null);
    builder.addDoubleProperty("Target X", () -> m_targetX, null);
    builder.addDoubleProperty("Target Y", () -> m_targetY, null);
  }
}