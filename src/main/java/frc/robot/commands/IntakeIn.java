// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeIn extends Command {
 //change later
  private Intake m_intakeMotor;
  /** Creates a new SpinForIntake. */
  public IntakeIn(Intake intake) {
    m_intakeMotor = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeMotor.setIntakeSpeed(0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeMotor.setIntakeSpeed(IntakeConstants.speed);
    SmartDashboard.putNumber("Intake Speed",IntakeConstants.speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeMotor.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
 