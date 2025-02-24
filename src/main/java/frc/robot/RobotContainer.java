// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.constants.IOConstants;
import frc.robot.constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


// swerve imports
import com.ctre.phoenix6.swerve.SwerveModule;
import frc.robot.commands.SwerveJoystick;
import frc.robot.subsystems.SwerveDrive;

// arm imports
import frc.robot.subsystems.Arm;
import frc.robot.commands.ArcadeArm;
import frc.robot.commands.ArmToPos;

// elevator imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.ArcadeElevator;
import frc.robot.commands.ElevatorToPosition;
// import frc.robot.commands.ElevatorPosition; -- as far as i know this file hasn't been tuned or used
// so it is commented out. See ElevatorToPosition for trapezoidal elevator
// import frc.robot.commands.ElevatorRatioTest; -- code used during testing

// pivot imports
import frc.robot.subsystems.Pivot;
import frc.robot.commands.ArcadePivot;
import frc.robot.commands.PIDForPivot;
// import frc.robot.commands.PivotToPosition; -- unused, use PID for Pivot instead

import frc.robot.commands.SpinArmOuttakeMotor;


import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;
import frc.robot.commands.RunIndexer;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeWithIndexer;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than commandsthe scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and  are defined here...

  public final SwerveDrive m_swerve = new SwerveDrive();
  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);


  private final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  private SendableChooser<Command> m_autoChooser;


  private final InstantCommand m_resetHeadingCommand = m_swerve.resetHeadingCommand();
  private final JoystickButton m_resetHeadingButton = new JoystickButton(m_driverJoystick, IOConstants.kResetHeadingButtonID);





  private final Arm m_arm = new Arm(); 
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);
  private final ArmToPos m_armToPos = new ArmToPos(m_arm, 0.781); // TODO: Change tick number
  private JoystickButton m_armToPosButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmButtonID);
  // use these for actual code
  private final JoystickButton m_armIntakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeIntakeButtonID);
  private final JoystickButton m_armOuttakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeOuttakeButtonID);


  private SpinArmOuttakeMotor m_spinArmOuttake = new SpinArmOuttakeMotor(m_arm, -0.7); // spin out is probably a negative speed, and this just spins it out
  private JoystickButton m_spinArmOuttakeRollersButton = new JoystickButton(m_secondJoystick, IOConstants.kArmOuttakeRollersButtonID);




  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);

  // Input of 15 means that the elevator will ideally move up by 15 inches. This was just chosen as a test.
  // this is a trapezoidal command!!
  private JoystickButton m_elevatorTestButton = new JoystickButton(m_secondJoystick, ElevatorConstants.kElevatorTestButtonID);
  private ElevatorToPosition m_elevatorTest = new ElevatorToPosition(m_elevator, 15);





  

  private Pivot m_pivot = new Pivot();
  private double m_speed = PivotConstants.kPivotSpeed; //change later

  // we actually only have 2 pivot positions -- the intake from the ground, and the stow upwards
  // intake from the ground is at approximately 0.1385 rotations
  // pivot stow is at approximately 0.8069 rotations

  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_secondJoystick);

  private PIDForPivot m_pivotPIDToStow = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  private PIDForPivot m_pivotPIDToIntake = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  private JoystickButton m_pivotButtonToStow = new JoystickButton(m_secondJoystick, PivotConstants.kPivotStowButtonID);
  private JoystickButton m_pivotButtonToIntake = new JoystickButton(m_secondJoystick, PivotConstants.kPivotIntakeButtonID);





  // begin intake/indexer
  private Intake m_intake = new Intake();
  private final double kIntakeIndexerSpeed = 0.6;
  private RunIntake m_intakeIn = new RunIntake(m_intake, kIntakeIndexerSpeed); // positive speed == intake in
  private RunIntake m_intakeOut = new RunIntake(m_intake, -kIntakeIndexerSpeed);

  private Indexer m_indexer = new Indexer();
  private RunIndexer m_indexerIn = new RunIndexer(m_indexer, kIntakeIndexerSpeed);
  private RunIndexer m_indexerOut = new RunIndexer(m_indexer, -kIntakeIndexerSpeed);

  // Same speed:
  // private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, kIntakeIndexerSpeed);
  // Different speed:
  // private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, 0.3, 0.5);

  private RunIntakeWithIndexer m_spinIntakeRollers = new RunIntakeWithIndexer(m_intake, m_indexer, kIntakeIndexerSpeed); // used for ground intake coral


  // note: these buttons are both not assigned and also missing the right IDs
  // private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeInButtonID);
  // private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeOutButtonID);
  // private JoystickButton m_indexerInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerInButtonID);
  // private JoystickButton m_indexerOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerOutButtonID);

  private JoystickButton m_groundIntakeCoralButton = new JoystickButton(m_secondJoystick, IOConstants.kGroundIntakeCoralButtonID);
  // end intake/indexer










  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    getTeleopCommand();
    m_autoChooser = AutoBuilder.buildAutoChooser();
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
    // see discord channel for button bindings

    
    // driver joystick bindings:
    m_resetHeadingButton.onTrue(m_resetHeadingCommand); // button 4, y button
    // to add - button 5 - left align to reef (vision)
    // to add - button 6 - right align to reef (vision)
    m_spinArmOuttakeRollersButton.whileTrue(m_spinArmOuttake); // button 7, spin arm outtake roller



    // button 8 -- ground intake coral
    // to my knowledge, this button, when pressed, is supposed to 
    // 1. make the pivot go down
    // 2. indefinitely spin the intake/indexer wheels
    // when released, this command is to move the pivot up, and also spin the indexer in for 1 second, then spin the indexer out for 3 seconds
    // NOTE this command (as of now) DOES NOT move the arm to the desired location to ground intake coral
  
    // lowkey unsure if this code works
    m_groundIntakeCoralButton.onTrue(
      m_pivotPIDToIntake.andThen(m_spinIntakeRollers)
    );

    
    // what it does is move the pivot up and at the same time index in for 1s then index out for 2s
    m_groundIntakeCoralButton.onFalse(
      Commands.parallel(
        m_pivotPIDToStow,
        m_indexerIn.withTimeout(1).andThen(
          m_indexerOut.withTimeout(2)
        )
      )
    );

    // end driver joystick bindings







    // m_elevatorPositionButton.whileTrue(m_elevatorPosition); TODO: Restore this
    m_elevatorTestButton.onTrue(m_elevatorTest);

    // make sure this doesn't accidently run
    // m_armToPosButton.whileTrue(m_armToPos);

    
    m_pivotButtonToStow.onTrue(m_pivotPIDToStow);
    m_pivotButtonToIntake.onTrue(m_pivotPIDToIntake);

    
  }




  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }



  private void getTeleopCommand() {
    m_swerve.setDefaultCommand(m_swerveJoystick);


    // m_arm.setDefaultCommand(m_arcadeArm);
    m_pivot.setDefaultCommand(m_arcadePivot);
    // m_elevator.setDefaultCommand(m_arcadeElevator);
    // m_elevator.setDefaultCommand(m_elevatorPosition);
  }
}
  
