// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ServoMovement extends Command {
  /** Creates a new ServoMovement. */

  private Climb m_climb;
  private double m_servoPos;
  public ServoMovement(Climb climb, double pos) {
    m_climb = climb;
    m_servoPos = pos;
    addRequirements(climb);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setServoPosition(m_servoPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (
      (m_climb.getServoPosition() - m_servoPos < ClimbConstants.kServoTolerance) ||
      (m_servoPos - m_climb.getServoPosition() < ClimbConstants.kServoTolerance)
    );
  }
}
