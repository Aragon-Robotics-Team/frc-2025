// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ArcadePivot;
import frc.robot.commands.PivotToPosition;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Arm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ArcadeArm;



import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeElevator;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and commands are defined here...
  //public final SwerveDrive m_swerve = new SwerveDrive();
  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);
  //private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  //private SendableChooser<Command> m_autoChooser;
  private Pivot m_pivot = new Pivot();
  private double m_speed = PivotConstants.kPivotSpeed; //change later
  private PivotToPosition m_pivotToA = new PivotToPosition(m_pivot, m_speed, PivotConstants.kPivotPositionToA);
  private PivotToPosition m_pivotToB = new PivotToPosition(m_pivot, m_speed, PivotConstants.kPivotPositionToB);
  private PivotToPosition m_pivotToC = new PivotToPosition(m_pivot, m_speed, PivotConstants.kPivotPositionToC);
  private JoystickButton m_pivotButtonToA = new JoystickButton(m_secondJoystick, PivotConstants.kButtonNumToA);
  private JoystickButton m_pivotButtonToB = new JoystickButton(m_secondJoystick, PivotConstants.kButtonNumToB);
  private JoystickButton m_pivotButtonToC = new JoystickButton(m_secondJoystick, PivotConstants.kButtonNumToC);

  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_secondJoystick);
  // private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  private SendableChooser<Command> m_autoChooser;
  
  private final Arm m_arm = new Arm();
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);
  

  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    getTeleopCommand();
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
    //return m_autoChooser.getSelected();
    return null; //temporary
  }

  private void getTeleopCommand() {
    m_swerve.setDefaultCommand(m_swerveJoystick);
    m_arm.setDefaultCommand(m_arcadeArm);
    m_elevator.setDefaultCommand(m_arcadeElevator);

  }
}