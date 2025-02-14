// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.ArcadeArm;
import frc.robot.commands.ArcadeElevator;
import frc.robot.commands.ArcadePivot;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.ElevatorRatioTest;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.PIDForPivot;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeWithIndexer;
import frc.robot.commands.SpinArmOuttakeMotor;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than commandsthe scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and  are defined here...
  //public final SwerveDrive m_swerve = new SwerveDrive();
  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);
  // private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  private Elevator m_elevator = new Elevator();

  private final ElevatorPosition m_elevatorPosition = new ElevatorPosition(m_elevator,42);
  private JoystickButton m_elevatorPositionButton = new JoystickButton(m_secondJoystick, 1);
  private SendableChooser<Command> m_autoChooser;
  // private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  // private SendableChooser<Command> m_autoChooser;

  private Intake m_intake = new Intake();
  private RunIntake m_intakeIn = new RunIntake(m_intake, 0.3); // positive speed == intake in
  private RunIntake m_intakeOut = new RunIntake(m_intake, -0.3);

  private Indexer m_indexer = new Indexer();
  private RunIndexer m_indexerIn = new RunIndexer(m_indexer, 0.3); //TODO: Change these speeds
  private RunIndexer m_indexerOut = new RunIndexer(m_indexer, -0.3);

  // Same speed:
  private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, 0.3);
  // Different speed:
  // private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, 0.3, 0.5);

  private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeInButtonID); //change number later
  private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeOutButtonID);
  private JoystickButton m_indexerInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerInButtonID);
  private JoystickButton m_indexerOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerOutButtonID);
  private JoystickButton m_intakeWithIndexerButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeWithIndexerButtonID);
  // Same speed:
  private final Arm m_arm = new Arm();
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);
  private final ArmToPos m_armToPos = new ArmToPos(m_arm, 0.781); // TODO: Change tick number

  private JoystickButton m_armToPosButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmButtonID);

  private final SpinArmOuttakeMotor m_intakeArmOuttakeMotor = new SpinArmOuttakeMotor(m_arm, 0.7);
  private final SpinArmOuttakeMotor m_outtakeArmOuttakeMotor = new SpinArmOuttakeMotor(m_arm, -0.7);
  private final JoystickButton m_intakeArmOuttakeButton = new JoystickButton(m_secondJoystick, 0);

  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);

  //Input of 15 means that the elevator will ideally move up by 15 inches. This was just chosen as a test.

  private JoystickButton m_elevatorTestButton = new JoystickButton(m_secondJoystick, ElevatorConstants.kElevatorTestButtonID);
  private ElevatorToPosition m_elevatorTest = new ElevatorToPosition(m_elevator, 15);

  private JoystickButton m_elevatorRatioTestButton = new JoystickButton(m_secondJoystick, 4);
  private ElevatorRatioTest m_elevatorRatioTestone = new ElevatorRatioTest(m_elevator, 0.35);

  private JoystickButton m_elevatorRatioTestButtonTwo = new JoystickButton(m_secondJoystick, 3);
  private ElevatorRatioTest m_elevatorRatioTesttwo = new ElevatorRatioTest(m_elevator, 0.4);

  private JoystickButton m_elevatorRatioTestButtonThree = new JoystickButton(m_secondJoystick, 2);
  private ElevatorRatioTest m_elevatorRatioTestthree = new ElevatorRatioTest(m_elevator, 0.45);
  
  private Pivot m_pivot = new Pivot();
  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_secondJoystick);

  private PIDForPivot m_pivotPIDToStow = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  private PIDForPivot m_pivotPIDToIntake = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  private JoystickButton m_pivotButtonToStow = new JoystickButton(m_secondJoystick, PivotConstants.kPivotStowButtonID);
  private JoystickButton m_pivotButtonToIntake = new JoystickButton(m_secondJoystick, PivotConstants.kPivotIntakeButtonID);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    getTeleopCommand();
    // m_autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Driving/Auto Chooser", m_autoChooser);
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

    // m_elevatorPositionButton.whileTrue(m_elevatorPosition); TODO: Restore this
    // m_armToPosButton.whileTrue(m_armToPos);
   
    m_intakeInButton.whileTrue(m_intakeIn);
    m_intakeOutButton.whileTrue(m_intakeOut);
    m_indexerInButton.whileTrue(m_intakeIn);
    m_indexerOutButton.whileTrue(m_intakeOut);
    m_intakeWithIndexerButton.whileTrue(m_intakeWithIndexer);
    m_armToPosButton.whileTrue(m_armToPos);
    m_elevatorPositionButton.whileTrue(m_elevatorPosition);
    m_elevatorTestButton.onTrue(m_elevatorTest);
    //m_elevatorRatioTestButton.whileTrue(m_elevatorRatioTestone);
    //m_elevatorRatioTestButtonTwo.whileTrue(m_elevatorRatioTesttwo);
    //m_elevatorRatioTestButtonThree.whileTrue(m_elevatorRatioTestthree);

    // m_pivotButtonToA.onTrue(m_pivotToA);
    // m_pivotButtonToB.onTrue(m_pivotToB);

    m_pivotButtonToStow.onTrue(m_pivotPIDToStow);
    m_pivotButtonToIntake.onTrue(m_pivotPIDToIntake);
    // m_armToPosButton.whileTrue(m_armToPos);
   
    m_intakeInButton.whileTrue(m_intakeIn);
    m_intakeOutButton.whileTrue(m_intakeOut);
    m_indexerInButton.whileTrue(m_intakeIn);
    m_indexerOutButton.whileTrue(m_intakeOut);
    m_intakeWithIndexerButton.whileTrue(m_intakeWithIndexer);
    m_armToPosButton.whileTrue(m_armToPos);
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
    // commenting this out for elevator testing so arm doesn't randomly trigger
    // m_arm.setDefaultCommand(m_arcadeArm);
    m_pivot.setDefaultCommand(m_arcadePivot);
    // comment it all out so that intake doesn't accidently trigger arm
    // m_arm.setDefaultCommand(m_arcadeArm);
    // m_swerve.setDefaultCommand(m_swerveJoystick);
    // m_elevator.setDefaultCommand(m_arcadeElevator);
    // m_elevator.setDefaultCommand(m_elevatorPosition);
  }
}
  
