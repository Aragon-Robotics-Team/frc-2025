// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.ArmToPos;
import frc.robot.commands.arm.SpinEndEffectorMotor;
import frc.robot.commands.elevator.ElevatorToPosition;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardAndPlace extends ParallelCommandGroup {
  /** Creates a new DriveForwardAndPlace. */
  public DriveForwardAndPlace(ArmToPos armToL1, ElevatorToPosition elevatorToGround, MoveForTime moveForTime, SpinEndEffectorMotor endEffectorOuttake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      armToL1,
      elevatorToGround,
      new SequentialCommandGroup(new WaitCommand(1), moveForTime),
      new SequentialCommandGroup(new WaitCommand(8.5), endEffectorOuttake.withTimeout(1)) // 7s move for time so add an extra half second for buffer
      // add an extra second so that the end effector doesn't outtake forever
    );
  }
}
