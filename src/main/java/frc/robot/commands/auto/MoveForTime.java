// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveForTime extends Command {
  private SwerveDrive m_swerve;
  private double m_goalTime; // recorded in seconds
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_turningSpeed;

  /** Creates a new MoveForTime. */
  public MoveForTime(SwerveDrive swerve, double goalTime, double xSpeed, double ySpeed, double turningSpeed) {
    m_swerve = swerve;
    m_goalTime = goalTime;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_turningSpeed = turningSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_turningSpeed, null));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, null));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() >= m_goalTime;
  }
}
