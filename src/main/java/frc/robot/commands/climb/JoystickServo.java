// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class JoystickServo extends Command {
  /** Creates a new JoystickServo. */
  private Joystick m_joystick;
  private Climb m_climb;
  public JoystickServo(Joystick joystick, Climb climb) {
    m_joystick = joystick;
    m_climb = climb;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.setServoPosition(0.598333); // copied from robot container
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.setServoPosition(m_climb.getServoPosition() + m_joystick.getRawAxis(0)*0.05); // 0.05 constant to stop servo from moving super fast
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
