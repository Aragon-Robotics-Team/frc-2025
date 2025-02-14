// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorToPosition extends Command {
  private Elevator m_elevator;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_start;
  private PIDController m_pid;


  public ElevatorToPosition(Elevator elevator, double goal) {
    m_elevator = elevator;

    //goal is inputted as inches on the elevator. The line below converts it to ticks.

    m_goal = new TrapezoidProfile.State(goal*ElevatorConstants.kTicksPerFoot/12, 0);

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.maxSpeed*ElevatorConstants.kTicksPerSecondPerSpeed, ElevatorConstants.maxAcceleration));
    m_timer = new Timer();
    m_pid = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start = new TrapezoidProfile.State(m_elevator.getElevatorPosition(), m_elevator.getSpeed());
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State idealState = m_profile.calculate(m_timer.get(), m_start, m_goal);
    double speed = idealState.velocity + m_pid.calculate(m_elevator.getElevatorPosition(), idealState.position);
    //speed is currently in ticks/s, so the divison converts it back to "motor" speed.
    m_elevator.setSpeed(speed/ElevatorConstants.kTicksPerSecondPerSpeed);

    //All constants are in ticks and seconds

    SmartDashboard.putNumber("Elevator/setspeed", speed);

    SmartDashboard.putNumber("Elevator/real velocity RPM", m_elevator.getSpeed());
    SmartDashboard.putNumber("Elevator/feedForward", idealState.velocity);
    SmartDashboard.putNumber("Elevator/velocity error", m_elevator.getSpeed() - idealState.velocity);
    SmartDashboard.putNumber("Elevator/position (Rotations)", m_elevator.getElevatorPosition());
    SmartDashboard.putNumber("Elevator/ideal position (ticks)", idealState.position);
    SmartDashboard.putNumber("Elevator/position error", m_elevator.getElevatorPosition() - idealState.position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(m_goal.position - m_elevator.getElevatorPosition()) < ElevatorConstants.kPositionDeadband && Math.abs(m_elevator.getSpeed()) < ElevatorConstants.kVelocityDeadband){
      return true;
    }
    return false;
  }
}
