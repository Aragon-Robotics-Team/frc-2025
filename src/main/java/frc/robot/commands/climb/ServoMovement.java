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
  private boolean kMoveVortex;
  public ServoMovement(Climb climb, double pos, boolean moveVortex) {
    m_climb = climb;
    m_servoPos = pos;
    kMoveVortex = moveVortex;
    addRequirements(climb);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Servo init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (kMoveVortex){
      m_climb.setClimbMotorSpeed(0.015); 
      // small constant in case the hex shaft gets stuck and the motor doesn't have enough power to move the shaft
    }
    m_climb.setServoPosition(m_servoPos);
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
