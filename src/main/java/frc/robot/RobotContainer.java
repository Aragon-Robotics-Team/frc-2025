// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.ArcadePivot;
import frc.robot.subsystems.Pivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveDrive m_swerve = new SwerveDrive();
  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);
  private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  private SendableChooser<Command> m_autoChooser;
  private Pivot m_pivot = new Pivot();
  private double m_speed = 0; //change later
  private ArcadePivot m_pivotToA = new ArcadePivot(m_pivot, m_speed, 0);
  private ArcadePivot m_pivotToB = new ArcadePivot(m_pivot, m_speed, 0);
  private ArcadePivot m_pivotToC = new ArcadePivot(m_pivot, m_speed, 0);
  private JoystickButton m_pivotButtonToA = new JoystickButton(m_secondJoystick, 0);
  private JoystickButton m_pivotButtonToB = new JoystickButton(m_secondJoystick, 0);
  private JoystickButton m_pivotButtonToC = new JoystickButton(m_secondJoystick, 0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Driving/Auto Chooser", m_autoChooser);
    // Configure the trigger bindings
    configureBindings();
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
    m_pivotButtonToA.onTrue(m_pivotToA);
    m_pivotButtonToB.onTrue(m_pivotToB);
    m_pivotButtonToC.onTrue(m_pivotToC);
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

  public Command getTeleopCommand() {
    m_swerve.setDefaultCommand(m_swerveJoystick);
    return null;
  }

  
}
