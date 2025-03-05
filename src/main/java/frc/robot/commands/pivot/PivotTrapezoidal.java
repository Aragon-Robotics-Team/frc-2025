// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotTrapezoidal extends Command {
  private Pivot m_pivot;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile m_profile;
  private Timer m_timer;
  private TrapezoidProfile.State m_start;
  private PIDController m_pid;
  /** Creates a new PivotTrapezoidal. */
  public PivotTrapezoidal(Pivot pivot, double goal) {
    m_pivot = pivot;

    //goal is inputted as ticks

    m_goal = new TrapezoidProfile.State(goal, 0); // this is now logged in ticks

    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(PivotConstants.kMaxSpeed, PivotConstants.kMaxAcceleration));

    m_timer = new Timer();
    m_pid = new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
    
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start = new TrapezoidProfile.State(m_pivot.getPivotPosition(), m_pivot.getSpeed());
    m_timer.restart();
    SmartDashboard.putNumber("Time", m_timer.get());
    SmartDashboard.putNumber("Goal", m_goal.position);
    System.out.println(m_goal.position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State idealState = m_profile.calculate(m_timer.get(), m_start, m_goal);
    double speed = m_pid.calculate(m_pivot.getPivotPosition(), idealState.position);
    //speed is currently in ticks/s, so the divison converts it back to "motor" speed.
    m_pivot.setPivotSpeed(speed);

    //All constants are in ticks and seconds

    /** SmartDashboard.putNumber("Pivot/setspeed", speed/ElevatorConstants.kTicksPerSecondPerSpeed);

    SmartDashboard.putNumber("Pivot/real velocity RPS", m_pivot.getSpeed());
    SmartDashboard.putNumber("Pivot/ideal velocity", idealState.velocity);
    SmartDashboard.putNumber("Pivot/velocity error", m_pivot.getSpeed() - idealState.velocity);
    SmartDashboard.putNumber("Pivot/position (ticks)", m_pivot.getPivotPosition());
    SmartDashboard.putNumber("Pivot/position command", idealState.position);
    SmartDashboard.putNumber("Pivot/position error", m_pivot.getPivotPosition() - idealState.position);
    // SmartDashboard.putNumber("Goal", m_goal.position);
    SmartDashboard.putNumber("Time", m_timer.get());**/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //SmartDashboard.putNumber("Time", m_timer.get());
    //m_pivot.setPivotSpeed(0); // reset
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
