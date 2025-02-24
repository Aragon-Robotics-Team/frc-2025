// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_indexer;

import java.nio.file.WatchEvent.Kind;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeWithIndexerJoystick extends Command {
  /** Creates a new RunIntakeWithIndexerJoystick. */

  private Intake m_intake;
  private Indexer m_indexer;
  private Joystick m_joystick;
  private boolean kIsIntake; // true if the rollers are supposed to be intaking, false if they're supposed to be outtaking
  // on a trigger it is only 0 -> 1 (not -1 -> 1) so this distinction needs to be made


  public RunIntakeWithIndexerJoystick(Joystick joystick, Intake intake, Indexer indexer, boolean isIntake) {
    m_joystick = joystick;
    m_intake = intake;
    m_indexer = indexer;
    kIsIntake = isIntake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kIsIntake){
      m_intake.setIntakeSpeed(m_joystick.getRawAxis(IntakeConstants.kManualOuttakeTriggerAxis));
      m_indexer.setIndexerSpeed(m_joystick.getRawAxis(IntakeConstants.kManualOuttakeTriggerAxis));
    } else {
      m_intake.setIntakeSpeed(-m_joystick.getRawAxis(IntakeConstants.kManualIntakeTriggerAxis));
      m_indexer.setIndexerSpeed(-m_joystick.getRawAxis(IntakeConstants.kManualIntakeTriggerAxis));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(0);
    m_indexer.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
