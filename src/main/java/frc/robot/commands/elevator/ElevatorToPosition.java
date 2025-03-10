// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

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

    // m_goal = new TrapezoidProfile.State(goal*ElevatorConstants.kTicksPerFoot/12, 0);
    m_goal = new TrapezoidProfile.State(goal, 0); // this is now logged in ticks

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(ElevatorConstants.kMaxSpeed*ElevatorConstants.kTicksPerSecondPerSpeed, ElevatorConstants.kMaxAcceleration));

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
    SmartDashboard.putNumber("Time", m_timer.get());
    SmartDashboard.putNumber("Goal", m_goal.position);
    System.out.println(m_goal.position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State idealState = m_profile.calculate(m_timer.get(), m_start, m_goal);
    double speed = m_pid.calculate(m_elevator.getElevatorPosition(), idealState.position);
    //speed is currently in ticks/s, so the divison converts it back to "motor" speed.
    m_elevator.setSpeed(speed);

    //All constants are in ticks and seconds

    SmartDashboard.putNumber("Elevator/setspeed", speed/ElevatorConstants.kTicksPerSecondPerSpeed);

    SmartDashboard.putNumber("Elevator/real velocity RPS", m_elevator.getSpeed());
    SmartDashboard.putNumber("Elevator/ideal velocity", idealState.velocity);
    SmartDashboard.putNumber("Elevator/velocity error", m_elevator.getSpeed() - idealState.velocity);
    SmartDashboard.putNumber("Elevator/position (ticks)", m_elevator.getElevatorPosition());
    SmartDashboard.putNumber("Elevator/position command", idealState.position);
    SmartDashboard.putNumber("Elevator/position error", m_elevator.getElevatorPosition() - idealState.position);
    // SmartDashboard.putNumber("Goal", m_goal.position);
    SmartDashboard.putNumber("Time", m_timer.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putNumber("Time", m_timer.get());
    m_elevator.setSpeed(0); // reset
  }

  public boolean atSetpoint(){
    return m_pid.atSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(Math.abs(m_goal.position - m_elevator.getElevatorPosition()) < ElevatorConstants.kPositionDeadband && Math.abs(m_elevator.getSpeed()) < ElevatorConstants.kVelocityDeadband){
    //   return true;
    // } 
    return false;
  }
}
