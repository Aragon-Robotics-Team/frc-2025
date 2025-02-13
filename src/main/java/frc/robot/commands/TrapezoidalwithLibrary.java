// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrapezoidalwithLibrary extends Command {
  /** Creates a new PIDToAngle. */

  private Pivot m_pivot;
  private double maxSpeed;
  private double maxAcceleration;  
  private double idealVelocity;
  private final TrapezoidProfile m_profile;
  private double goal;
  private TrapezoidProfile.State m_goal;
  private TrapezoidProfile.State m_current;
  private Timer m_timer;

  public TrapezoidalwithLibrary(Pivot pivot, double goal, double maxSpeed, double maxAcceleration) {
    m_pivot = pivot;
    this.maxSpeed = maxSpeed;
    this.maxAcceleration = maxAcceleration;
    m_profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxSpeed, maxAcceleration));
    m_goal = new TrapezoidProfile.State(goal, 0);
    m_current = new TrapezoidProfile.State();
    this.goal = goal;
    m_timer = new Timer();
    addRequirements(m_pivot);
  }

  int checkpoint = 0;
  double distanceAfterAcceleration;
  double slowDownStartTime;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_current = new TrapezoidProfile.State(m_pivot.getRotations(), m_pivot.getSpeed());
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TrapezoidProfile.State idealState = m_profile.calculate(m_timer.get(), m_current, m_goal);
    double speed = idealState.velocity + PivotConstants.trapezoidalKP*(idealState.position - m_pivot.getRotations()) + PivotConstants.trapezoidalKD*(idealState.velocity - m_pivot.getSpeed());
    m_pivot.getPivotSim().setInputVoltage(12*speed);
    SmartDashboard.putNumber("Pivot Speed", m_pivot.getSpeed());
    SmartDashboard.putNumber("Pivot Speed Error", m_pivot.getSpeed() - idealVelocity);
    SmartDashboard.putNumber("Rotations:", m_pivot.getRotations()); 
    SmartDashboard.putNumber("Voltage:", m_pivot.getVoltage()); 
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
      return false;
    }

    return false;
  }
}
