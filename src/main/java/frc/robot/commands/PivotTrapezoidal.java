// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PivotTrapezoidal extends Command {
  /** Creates a new PIDToAngle. */

  private Pivot m_pivot;
  private double goal; // (this is measured in rotations)
  private double maxSpeed;
  private double accelTime;  
  private Timer m_timer;
  private double idealVelocity;

  public PivotTrapezoidal(Pivot pivot, double goal, double maxSpeed, double accelTime) {
    m_pivot = pivot;
    this.goal = goal;
    this.maxSpeed = maxSpeed;
    this.accelTime = accelTime;
    this.m_timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_pivot);
  }

  int checkpoint = 0;
  double distanceAfterAcceleration;
  double slowDownStartTime;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(checkpoint == 0){
      distanceAfterAcceleration = m_pivot.getRotations();
    }
    if(checkpoint == 1){
      slowDownStartTime = m_timer.get();
    }
    // speed is proportional to distance to goal
    // this is a simple PID (utilizing only the P)

    if(m_timer.get() < accelTime){
      idealVelocity = m_timer.get()*maxSpeed/accelTime;
    } else if (m_pivot.getRotations() < goal - distanceAfterAcceleration){
      checkpoint = 1;
      idealVelocity = maxSpeed;
    }
    else if (m_pivot.getRotations() < goal) {
      checkpoint = 2;
      idealVelocity = (slowDownStartTime + accelTime - m_timer.get())*maxSpeed/accelTime;
    }
    else {
      checkpoint = 3;
      idealVelocity = 0;
    }

    double velocity = idealVelocity;
    m_pivot.getPivotSim().setInputVoltage(12*velocity);

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
