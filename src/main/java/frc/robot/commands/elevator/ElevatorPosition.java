// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorPosition extends Command {
  private static final class Config {
    private static final double kP = 0.035;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
  }

  private PIDController m_pid = new PIDController(Config.kP, Config.kI, Config.kD);
  private Elevator m_elevator;
  private double m_initialPosition;
  private double m_currentPosition;
  private double m_error;


  /** Creates a new ElevatorPosition. */
  public ElevatorPosition(Elevator elevator, double goal) {
    m_elevator = elevator;
    m_pid.setSetpoint(goal);
    m_pid.setTolerance(0.005);
  
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_initialPosition = m_elevator.getElevatorPosition();

  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentPosition = m_elevator.getElevatorPosition();
    //m_currentPosition = m_currentPosition - m_initialPosition;
    double setSpeed = m_pid.calculate(m_currentPosition, m_pid.getSetpoint());
    // if (setSpeed > 0.5)
    // {
    //   setSpeed = 0.5;
    // }
    Logger.recordOutput("currentPosition", m_currentPosition);
    Logger.recordOutput("Setpoint", m_pid.getSetpoint());
    m_elevator.setSpeed(setSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pid.atSetpoint();
  }
}
