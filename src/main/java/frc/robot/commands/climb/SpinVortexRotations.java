// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import frc.robot.Constants.ClimbConstants; // capital constants

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinVortexRotations extends Command {
  /** Creates a new SpinVortex. */
  private Climb m_climb;
  private double m_speed; 
  private double m_rotations; // amount of rotations to spin

  public SpinVortexRotations(Climb climb, double speed, double rotations) {

    m_climb = climb;
    m_speed = speed;
    m_rotations = rotations;
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("We starting the cage thing");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Climb Motor Rotations", m_climb.getMotorRotations());
    m_climb.setClimbMotorSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climb.setClimbMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
      Math.abs(m_rotations - m_climb.getMotorRotations()) < ClimbConstants.kVortexRotationsTolerance
    );
  }
}
