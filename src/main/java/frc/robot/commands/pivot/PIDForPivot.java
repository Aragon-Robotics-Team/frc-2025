// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDForPivot extends Command {
  /** Creates a new PIDForPivot. */
  private PIDController m_pid = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
  private Pivot m_pivot;
  private double m_goal; // this goal should be in rotations -- a number from 0.672 to 0.999
  private double speed;

  public PIDForPivot(Pivot pivot, double goal) {
    m_pivot = pivot;
    m_goal = goal;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // NOTE
    // later, we should look into "Setting a Max Integrator Value" (see wpilib docs) which basically
    // limits the integral output value to stop excessive wind up/over shoot
    m_pid.setTolerance(PivotConstants.kPivotPIDErrorTolerance, PivotConstants.kPivotDerivativeTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = m_pid.calculate(m_pivot.getPivotPosition(), m_goal);
    m_pivot.setPivotSpeed(speed);

    // see Pivot.java subsystem
    // SmartDashboard.putNumber("Pivot Speed", m_pid.calculate(m_pivot.getPivotPosition(), m_goal));
    SmartDashboard.putNumber("Rotations", m_pivot.getPivotPosition());
    SmartDashboard.putNumber("Goal Rotations", m_goal);
    SmartDashboard.putNumber("Pivot Rotations Error", Math.max(m_goal, m_pivot.getPivotPosition()) - Math.min(m_pivot.getPivotPosition(), m_goal));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_pid.atSetpoint()) {
      return true;
    }
    return false;
  }
}
