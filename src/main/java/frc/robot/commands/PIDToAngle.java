// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToAngle extends Command {
  /** Creates a new PIDToAngle. */

  private Pivot m_pivot;
  private double goal; // (this is measured in rotations)

  public PIDToAngle(Pivot pivot, double goal) {
    m_pivot = pivot;
    this.goal = goal;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // speed is proportional to distance to goal
    // this is a simple PID (utilizing only the P)

    if (PivotConstants.kP*(goal - m_pivot.getRotations()) > 1){
      m_pivot.setPivotSpeed(1);
      SmartDashboard.putNumber("Pivot Speed", 1);
      SmartDashboard.putNumber("Rotations:", m_pivot.getRotations()); 
      m_pivot.getPivotSim().setInputVoltage(12);

    } else {
      m_pivot.setPivotSpeed(PivotConstants.kP*(goal - m_pivot.getRotations()));
      SmartDashboard.putNumber("Pivot Speed", PivotConstants.kP*(goal - m_pivot.getRotations()));
      SmartDashboard.putNumber("Rotations:", m_pivot.getRotations()); 

      // update the motor sim
      m_pivot.getPivotSim().setInputVoltage(12*PivotConstants.kP*(goal - m_pivot.getRotations()));
    }
    m_pivot.getPivotSim().update(0.02); // 20ms 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // command is finished when we're within tolerance

    if (Math.max(m_pivot.getRotations(), goal) - Math.min(m_pivot.getRotations(), goal) < PivotConstants.kToleranceRotations){
      return true;
    }

    return false;
  }
}
