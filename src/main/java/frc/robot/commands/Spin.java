
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Motor;

public class Spin extends Command {
  /** Creates a new Spin. */
  private Motor m_motor;
  private Joystick m_joystick;
  private double speed;
  private final double kSpeedMultiplier = 0.05;

  public Spin(Joystick joystick, Motor motor) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_motor = motor;
    m_joystick = joystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_motor.setMotorSpeed(0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = m_joystick.getRawAxis(1) * kSpeedMultiplier;
    m_motor.setMotorSpeed(speed);
    

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
   m_motor.setMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
