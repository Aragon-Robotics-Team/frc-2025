// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.EndEffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinEndEffectorMotor extends Command {
  /** Creates a new SpinArmOuttakeMotor. */
  private EndEffector m_endEffector;
  private double m_speed;
  private boolean kJoystickControl; // true if we're controlling this with a joystick false if with just a constant speed
  // my bad gang this is actually terrible code
  private Joystick m_joystick;
  
  public SpinEndEffectorMotor(EndEffector endEffector, double speed, boolean joystickControl, Joystick joystick) {
    m_endEffector = endEffector;
    m_speed = speed;
    kJoystickControl = joystickControl;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kJoystickControl){
      m_endEffector.spinArmOuttakeMotor(m_joystick.getRawAxis(ArmConstants.kEndEffectorMotorOverrideAxis));
    } else {
      m_endEffector.spinArmOuttakeMotor(m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_endEffector.spinArmOuttakeMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
