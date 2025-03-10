// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriftSwerveJoystick;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.TurnToTagAngle;
import frc.robot.subsystems.DriftSwerveDrive;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // public final DriftSwerveDrive m_swerve = new DriftSwerveDrive();
  private final Vision m_vision = new Vision();
  public final SwerveDrive m_swerve = new SwerveDrive(m_vision);
  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);

  private final JoystickButton m_1stTagButton = new JoystickButton(m_driverJoystick, 1);
  private final JoystickButton m_2ndTagButton = new JoystickButton(m_driverJoystick, 2);
  private final JoystickButton m_3rdTagButton = new JoystickButton(m_driverJoystick, 3);
  private final JoystickButton m_4thTagButton = new JoystickButton(m_driverJoystick, 4);
  private final JoystickButton m_5thTagButton = new JoystickButton(m_driverJoystick, 5);
  private final JoystickButton m_6thTagButton = new JoystickButton(m_driverJoystick, 6);
  private final JoystickButton m_centerToTagButton = new JoystickButton(m_driverJoystick, 7);

  private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick, m_vision, m_1stTagButton, m_2ndTagButton, m_3rdTagButton, m_4thTagButton, m_5thTagButton, m_6thTagButton, m_centerToTagButton);
  // private final DriftSwerveJoystick m_swerveJoystick = new DriftSwerveJoystick(m_swerve, m_driverJoystick, m_vision, m_6thTagButton, m_5thTagButton, m_4thTagButton, m_3rdTagButton, m_2ndTagButton, m_1stTagButton, m_centerToTagButton);
  private SendableChooser<Command> m_autoChooser;
  // private final TurnToTagAngle m_turnToTagAngle = new TurnToTagAngle(m_swerve, 6);
  // private final JoystickButton m_turnToTag6Button = new JoystickButton(m_driverJoystick, 1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    getTeleopCommand();
    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Driving/Auto Chooser", m_autoChooser);
    // Configure the trigger bindings
    configureBindings();
    SmartDashboard.putData("Swerve/Odo/Reset_Heading", new InstantCommand(() -> m_swerve.resetHeading()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // m_turnToTag6Button.whileTrue(m_turnToTagAngle);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }

  private void getTeleopCommand() {
    m_swerve.setDefaultCommand(m_swerveJoystick);
  }

  
}
