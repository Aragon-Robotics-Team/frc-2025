// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotToPosition extends Command {
  private Pivot m_pivot;
  private double m_speed;
  private double m_goal;
  private double m_currentPosition;
  private double m_initialPosition;

  /** Creates a new ArcadePivot. */
  public PivotToPosition(Pivot pivot, double speed, double goal) {
    m_pivot = pivot;
    m_speed = speed;
    m_goal = goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pivot.setPivotSpeed(0);
    m_initialPosition = m_pivot.getPivotPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setPivotSpeed(m_speed);
    SmartDashboard.putNumber("Pivot Speed", m_speed);
    m_currentPosition = m_pivot.getPivotPosition() - m_initialPosition;
    SmartDashboard.putNumber("Rotations:", m_currentPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_currentPosition - m_goal < 0.01 && m_currentPosition - m_goal > -0.01) {
      return true;
    }
    return false;
  }
}
