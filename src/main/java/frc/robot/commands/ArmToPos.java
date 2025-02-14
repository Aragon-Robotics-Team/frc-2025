// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmToPos extends Command {
  private double m_pos;
  private Arm m_arm;
  
  // Creates a PIDController with gains kP, kI, and kD
  PIDController pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  
  /** Creates a new ArmToPos. */
  public ArmToPos(Arm arm, double pos) {
    m_pos = pos;
    m_arm = arm;
    addRequirements(m_arm); 
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setSpeed(pid.calculate(m_arm.getArmPos(), m_pos));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if(Math.abs(m_pos - m_arm.getArmPos()) < 0.1){
    return true;
  }
   return false;
  }
}
