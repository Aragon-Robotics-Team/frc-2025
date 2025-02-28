// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorDealgae extends Command {
  /** Creates a new ElevatorDealgae. */
  private Elevator m_elevator;
  private double m_goal;
  private ElevatorToPosition m_elevatorToHeight;
  public ElevatorDealgae(Elevator elevator) {
    m_elevator = elevator;
    if (Math.abs(m_elevator.getElevatorPosition() - ElevatorConstants.kL3ElevatorHeight) <= ElevatorConstants.kDealgaeThreshold) {
        m_goal = ElevatorConstants.kL3DealgaeElevatorHeight;
      } else if (Math.abs(m_elevator.getElevatorPosition() - ElevatorConstants.kL2ElevatorHeight) <= ElevatorConstants.kDealgaeThreshold) {
        m_goal = ElevatorConstants.kL2DealgaeElevatorHeight;
      }
    m_elevatorToHeight = new ElevatorToPosition(m_elevator, m_goal);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
