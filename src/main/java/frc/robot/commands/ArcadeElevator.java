// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ArcadeElevator extends Command {
  private static final class Config {
    public static final int kElevatorYAxis = 5;
    public static final double kElevatorMultiplier = Math.PI/26;
  }
    private Joystick m_joystick;
    private Elevator m_elevator;
  /** Creates a new ArcadeElevator. */
  public ArcadeElevator(Joystick joystick, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystick = joystick;
    m_elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setSpeed(m_joystick.getRawAxis(Config.kElevatorYAxis) * Config.kElevatorMultiplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}