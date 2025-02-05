// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnToTagAngle extends Command {

  private SwerveDrive m_drive;
  private PIDController m_pid = new PIDController(DriveConstants.kTurnToAngleP, DriveConstants.kTurnToAngleI, DriveConstants.kTurnToAngleD);
  private double m_turningSpeed;
  private double m_targetAngle;
  private double m_currentAngle;
  private int m_targetID;

  /** Creates a new TurnToTagAngle. */
  public TurnToTagAngle(SwerveDrive drive, int targetID) {
    m_drive = drive;
    m_targetID = m_targetID;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turningSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentAngle = m_drive.getAngleDegrees();
    switch (m_targetID){
      case 6:
        m_targetAngle = VisionConstants.kTag6Angle;
        break;
      case 7:
        m_targetAngle = VisionConstants.kTag7Angle;
        break;
      case 8:
        m_targetAngle = VisionConstants.kTag8Angle;
        break;
      case 9: 
        m_targetAngle = VisionConstants.kTag9Angle;
        break;
      case 10:
        m_targetAngle = VisionConstants.kTag10Angle;
        break;
      case 11:
        m_targetAngle = VisionConstants.kTag11Angle;
        break;
      case 17:
        m_targetAngle = VisionConstants.kTag17Angle;
        break;
      case 18:
        m_targetAngle = VisionConstants.kTag18Angle;
        break;
      case 19:
        m_targetAngle = VisionConstants.kTag19Angle;
        break;
      case 20:
        m_targetAngle = VisionConstants.kTag20Angle;
        break;
      case 21:
        m_targetAngle = VisionConstants.kTag21Angle;
        break;
      case 22:
        m_targetAngle = VisionConstants.kTag22Angle;
        break;
    }
    m_turningSpeed = m_pid.calculate(m_currentAngle, m_targetAngle);
    m_drive.setTurningSpeed(m_turningSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turningSpeed = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
