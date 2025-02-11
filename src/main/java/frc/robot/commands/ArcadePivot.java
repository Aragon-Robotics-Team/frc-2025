// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadePivot extends Command {
  private Pivot m_pivot;
  private Joystick m_joystick;
  /** Creates a new ArcadePivot. */
  public ArcadePivot(Pivot pivot, Joystick joystick) {
    m_pivot = pivot;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_pivot.setPivotSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setPivotSpeed(m_joystick.getRawAxis(0));
    SmartDashboard.putNumber("Pivot Speed", m_joystick.getRawAxis(0));
    SmartDashboard.putNumber("Rotations", m_pivot.getSimPivotPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_pivot.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
