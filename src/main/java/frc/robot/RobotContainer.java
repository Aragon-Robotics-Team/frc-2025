// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.arm.ArcadeArm;
import frc.robot.commands.arm.ArmToPos;
import frc.robot.commands.arm.SpinEndEffectorMotor;
import frc.robot.commands.elevator.ArcadeElevator;
import frc.robot.commands.elevator.ElevatorToPosition;
import frc.robot.commands.intake_indexer.RunIndexer;
import frc.robot.commands.intake_indexer.RunIntake;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexer;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexerJoystick;
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.pivot.PIDForPivot;
import frc.robot.constants.IOConstants;
// arm imports
import frc.robot.subsystems.Arm;
// elevator imports
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
// pivot imports
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.SwerveDrive;


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


  private final JoystickButton m_elevatorArmManualControlButton = new JoystickButton(m_secondJoystick, IOConstants.kElevatorArmManualOverrideButtonID); // 7
  private final JoystickButton m_pivotArmManualControlButton = new JoystickButton(m_secondJoystick, IOConstants.kPivotArmManualOverrideButtonID); // 8 (i think)





  private final JoystickButton m_L1ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL1ScoringButtonID);
  private final JoystickButton m_L2ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL2ScoringButtonID);
  private final JoystickButton m_L3ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL3ScoringButtonID);
  private final JoystickButton m_L4ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL4ScoringButtonID);


  private final Arm m_arm = new Arm(); 
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);


  // unused
  /*
  private final ArmToPos m_armToPos = new ArmToPos(m_arm, 0.781); // TODO: Change tick number
  private JoystickButton m_armToPosButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmButtonID);
  // use these for actual code
  private final JoystickButton m_armIntakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeIntakeButtonID);
  private final JoystickButton m_armOuttakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeOuttakeButtonID);
  */


  private final ArmToPos m_armToL1 = new ArmToPos(m_arm, 0); // todo: change all these constants
  private final ArmToPos m_armToL2 = new ArmToPos(m_arm, 0); // see that google sheet where i did that thing
  private final ArmToPos m_armToL3 = new ArmToPos(m_arm, 0);
  private final ArmToPos m_armToL4 = new ArmToPos(m_arm, 0);

  private final ArmToPos m_armToSubstationIntake = new ArmToPos(m_arm, 0);
  private final ArmToPos m_armToGroundIntake = new ArmToPos(m_arm, 0); // probably used in different command

  private final JoystickButton m_stowArmButton = new JoystickButton(m_secondJoystick, IOConstants.kStowArmButtonID);
  private final JoystickButton m_substationIntakeArmButton = new JoystickButton(m_secondJoystick, IOConstants.kArmToSubstationButtonID);


  private final EndEffector m_endEffector = new EndEffector();
  private SpinEndEffectorMotor m_spinEndEffector = new SpinEndEffectorMotor(m_endEffector, -0.7, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
  private JoystickButton m_spinEndEffectorButton = new JoystickButton(m_secondJoystick, IOConstants.kArmOuttakeRollersButtonID);

  private SpinEndEffectorMotor m_joystickOverrideSpinEndEffector = new SpinEndEffectorMotor(m_endEffector, 0, true, m_secondJoystick); 
  // lowkey terrible coding practice
  // note that the speed here doesn't matter so I'm not going to fret about that too much




  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);

  // Input of 15 means that the elevator will ideally move up by 15 inches. This was just chosen as a test.
  // this is a trapezoidal command!!
  private JoystickButton m_elevatorTestButton = new JoystickButton(m_secondJoystick, ElevatorConstants.kElevatorTestButtonID);
  private ElevatorToPosition m_elevatorTest = new ElevatorToPosition(m_elevator, 15);


  // todo -- get constants
  private ElevatorToPosition m_elevatorToL2 = new ElevatorToPosition(m_elevator, 0);
  private ElevatorToPosition m_elevatorToL3 = new ElevatorToPosition(m_elevator, 0);
  private ElevatorToPosition m_elevatorToL4 = new ElevatorToPosition(m_elevator, 0);

  private ElevatorToPosition m_elevatorToSubstationIntake = new ElevatorToPosition(m_elevator, 0);
  private ElevatorToPosition m_elevatorToGround = new ElevatorToPosition(m_elevator, 0); // reset elevator position




  

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

  private RunIntakeWithIndexerJoystick m_manualOuttakeIndexerIntake = new RunIntakeWithIndexerJoystick(m_secondJoystick, m_intake, m_indexer, false);
  private RunIntakeWithIndexerJoystick m_manualIntakeIndexerIntake = new RunIntakeWithIndexerJoystick(m_secondJoystick, m_intake, m_indexer, true);

  // note: these buttons are both not assigned and also missing the right IDs
  // private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeInButtonID);
  // private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeOutButtonID);
  // private JoystickButton m_indexerInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerInButtonID);
  // private JoystickButton m_indexerOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerOutButtonID);

  private JoystickButton m_groundIntakeCoralButton = new JoystickButton(m_secondJoystick, IOConstants.kGroundIntakeCoralButtonID);
  // end intake/indexer

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    bindSubsystemCommands();
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
    m_spinEndEffectorButton.whileTrue(m_spinEndEffector); // button 7, spin arm outtake roller



    // button 8 -- ground intake coral
    // to my knowledge, this button, when pressed, is supposed to 
    // 1. make the pivot go down
    // 2. indefinitely spin the intake/indexer wheels
    // when released, this command is to move the pivot up, and also spin the indexer in for 1 second, then spin the indexer out for 3 seconds
    // NOTE this command (as of now) DOES NOT move the arm to the desired location to ground intake coral

    // --> this might actually be a good idea if arm is manually moved
  
    // lowkey unsure if this code works
    m_groundIntakeCoralButton.onTrue(
      m_pivotPIDToIntake.andThen(m_spinIntakeRollers)
    );

    // TODO: Change seconds
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

/////////////////////////////////////////////////////////////

    // start operator joystick bindings
    // for the joystick axes, once buttons 7/8 are pressed, manual arm control happens
    // same with pivot/elevator override

    // NOTE: I may need to fix these commands (and/or other commands) so that the manual override can interrupt other commands
    // button id 7 (back left button), allows manual control of elevator/arm
    // also allows for manual intake/outtake
    m_elevatorArmManualControlButton.whileTrue(
      Commands.parallel(
        m_arcadeElevator,
        m_arcadeArm,
        m_manualOuttakeIndexerIntake
      )
    );

    // button id 8 (back right button) allows manual control of arm/pivot
    m_pivotArmManualControlButton.whileTrue(
      Commands.parallel(
        m_arcadePivot,
        m_arcadeArm,
        m_manualIntakeIndexerIntake,
        m_joystickOverrideSpinEndEffector
      )
    );

    // L1-4 scoring
    // gonna assume it takes <1.5s to score that piece
    // finally, reset position by going back to ArmToGroundIntake position
    m_L1ScoringButton.onTrue(
      m_armToL1.andThen(m_spinEndEffector.withTimeout(1.5)).andThen(m_armToGroundIntake)
    );


    //************************************************************** */
    // MASSIVE TODO - make sure that this works and doesnt break metal
    //************************************************************** */
    m_L2ScoringButton.onTrue(
      Commands.parallel(
        m_elevatorToL2,
        m_armToL2
      ).andThen(
        m_spinEndEffector.withTimeout(1.5)
      ).andThen(
        Commands.parallel(
          m_armToGroundIntake,
          m_elevatorToGround
        )
      )
    );

    // L3
    m_L3ScoringButton.onTrue(
      Commands.parallel(
        m_elevatorToL3,
        m_armToL3
      ).andThen(
        m_spinEndEffector.withTimeout(1.5)
      ).andThen(
        Commands.parallel(
          m_armToGroundIntake,
          m_elevatorToGround
        )
      )
    );

    // L4
    m_L4ScoringButton.onTrue(
      Commands.parallel(
        m_elevatorToL4,
        m_armToL4
      ).andThen(
        m_spinEndEffector.withTimeout(1.5)
      ).andThen(
        Commands.parallel(
          m_armToGroundIntake,
          m_elevatorToGround
        )
      )
    );


    // bind buttons 5, 6
    m_stowArmButton.onTrue(m_armToGroundIntake); // button 5 -- stow arm
    m_substationIntakeArmButton.onTrue(m_armToSubstationIntake); // button 6 -- move arm to substation intake position
  }




  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }



  private void bindSubsystemCommands() {
    m_swerve.setDefaultCommand(m_swerveJoystick);


    // note: there are really no default commands 
    // instead, the code should do everything for us
    // when a manual override is needed, a button is already pressed
    // thus, see the button binding on true part for that
    // (note to self): I may get a watchdog error for this in which case I'll bind null or something

    // m_arm.setDefaultCommand(m_arcadeArm);
    // m_pivot.setDefaultCommand(m_arcadePivot);
    // m_elevator.setDefaultCommand(m_arcadeElevator);
    // m_elevator.setDefaultCommand(m_elevatorPosition);
  }
}
  
