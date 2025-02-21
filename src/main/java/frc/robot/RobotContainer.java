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
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.ArcadeArm;
import frc.robot.commands.ArcadeElevator;
import frc.robot.commands.ArmToPos;
import frc.robot.commands.ElevatorToPosition;
import frc.robot.commands.SwerveJoystick;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ElevatorPosition;
import frc.robot.commands.ElevatorRatioTest;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.commands.SpinArmOuttakeMotor;

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
  private final JoystickButton m_armIntakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeIntakeButtonID);
  private final JoystickButton m_armOuttakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeOuttakeButtonID);
  // private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  // private SendableChooser<Command> m_autoChooser;


  private final Arm m_arm = new Arm();
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);


  private final ArmToPos m_armToPos = new ArmToPos(m_arm, 0.781); // TODO: Change rotation number
  
  private JoystickButton m_armToPosButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmButtonID);


  private final SpinArmOuttakeMotor m_intakeArmOuttakeMotor = new SpinArmOuttakeMotor(m_arm, 0.7);
  private final SpinArmOuttakeMotor m_outtakeArmOuttakeMotor = new SpinArmOuttakeMotor(m_arm, -0.7);

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
    
    // make sure this doesn't accidently run
    // m_armToPosButton.whileTrue(m_armToPos);

    m_elevatorTestButton.onTrue(m_elevatorTest);
    //m_elevatorRatioTestButton.whileTrue(m_elevatorRatioTestone);
    //m_elevatorRatioTestButtonTwo.whileTrue(m_elevatorRatioTesttwo);
    //m_elevatorRatioTestButtonThree.whileTrue(m_elevatorRatioTestthree);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return m_autoChooser.getSelected();
    return null;
  }

  private void getTeleopCommand() {

    // commenting this out for elevator testing so arm doesn't randomly trigger
    // m_arm.setDefaultCommand(m_arcadeArm);
    //m_swerve.setDefaultCommand(m_swerveJoystick);
    // m_elevator.setDefaultCommand(m_arcadeElevator);
    // m_elevator.setDefaultCommand(m_elevatorPosition);
  }
}