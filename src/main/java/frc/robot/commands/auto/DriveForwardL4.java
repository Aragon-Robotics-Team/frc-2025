// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.arm.ArmToPos;
import frc.robot.commands.arm.SpinEndEffectorMotor;
import frc.robot.commands.elevator.ElevatorToPosition;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.SwerveDrive;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveForwardL4 extends ParallelCommandGroup {
  /** Creates a new DriveForwardAndPlace. */
  public DriveForwardL4(SwerveDrive swerve, Arm arm, Elevator elevator, EndEffector endEffector, Joystick secondJoystick) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ElevatorToPosition(elevator, ElevatorConstants.kL4ElevatorHeight),
      // new SequentialCommandGroup(new WaitCommand(0.2), new ArmToPos(arm, ArmConstants.kL4ArmTickPosition)),
      // new MoveForTime(swerve, 8, 0, 0.5, 0)
      // new SequentialCommandGroup(new WaitCommand(1), new MoveForTime(swerve, 8, 0.3, 0, 0)),
      // new SequentialCommandGroup(new WaitCommand(8.5), (new SpinEndEffectorMotor(endEffector, -0.9, isScheduled(), secondJoystick)).withTimeout(1)) // 7s move for time so add an extra half second for buffer
      // add an extra second so that the end effector doesn't outtake forever
    );
  }
}
