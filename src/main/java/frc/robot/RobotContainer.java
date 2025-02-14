// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeArm;
import frc.robot.commands.ArcadeElevator;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.SpinArmOuttakeMotor;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and commands are defined here...
  // public final SwerveDrive m_swerve = new SwerveDrive();
  // private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);
  private final JoystickButton m_button = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeJoystickButton);
  // private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  // private SendableChooser<Command> m_autoChooser;

  private final Arm m_arm = new Arm();
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);
  private final SpinArmOuttakeMotor m_spinArmOuttakeMotor = new SpinArmOuttakeMotor(m_arm);
  
  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    getTeleopCommand();
    //m_autoChooser = AutoBuilder.buildAutoChooser();
    //SmartDashboard.putData("Driving/Auto Chooser", m_autoChooser);
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
   m_button.whileTrue(m_spinArmOuttakeMotor);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
    //return m_autoChooser.getSelected();
  }

  private void getTeleopCommand() {
    m_arm.setDefaultCommand(m_arcadeArm);
    // m_swerve.setDefaultCommand(m_swerveJoystick);
    m_elevator.setDefaultCommand(m_arcadeElevator);

  }
}