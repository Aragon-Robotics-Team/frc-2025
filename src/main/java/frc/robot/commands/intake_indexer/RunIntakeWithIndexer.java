// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake_indexer;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RunIntakeWithIndexer extends ParallelCommandGroup {
  /** Creates a new RunIntakeWithIndexer. */
  public RunIntakeWithIndexer(Intake intake, Indexer indexer, double intakeSpeed, double indexerSpeed) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunIntake(intake, intakeSpeed),
      new RunIndexer(indexer, indexerSpeed)
    );
  }

  public RunIntakeWithIndexer(Intake intake, Indexer indexer, double speed){
    addCommands(
      new RunIntake(intake, speed),
      new RunIndexer(indexer, speed)
    );
  }
}
