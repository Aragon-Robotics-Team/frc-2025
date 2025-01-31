// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotToAngle extends Command {
  /** Creates a new PivotToAngle. */
  private Pivot m_pivot;
  private double kgoal;

  // here, goal is a number in rotations (so this lines up well with pivot)
  public PivotToAngle(Pivot pivot, double goal) {
    m_pivot = pivot;
    kgoal = goal;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.setPivotSpeed(PivotConstants.kPivotSpeed); // yippee
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.setPivotSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // THIS IS WRONG

    if ((m_pivot.getRotations() - kgoal < PivotConstants.kToleranceRotations) ||
        (kgoal - m_pivot.getRotations() < PivotConstants.kToleranceRotations)){
        return true;
    }
    
    return false;
  }
}
