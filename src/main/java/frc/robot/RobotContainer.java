// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.SwerveJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.constants.ArmConstants;
import frc.robot.constants.IOConstants;

import frc.robot.subsystems.SwerveDrive;

// arm imports
import frc.robot.subsystems.Arm;
import frc.robot.commands.arm.ArcadeArm;
import frc.robot.commands.arm.ArmToPos;
import frc.robot.commands.arm.SpinEndEffectorMotor;
import frc.robot.commands.auto.DriveForwardAndPlace;
import frc.robot.commands.auto.MoveForTime;
import frc.robot.subsystems.EndEffector;

// elevator imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.elevator.ArcadeElevator;
import frc.robot.commands.elevator.ElevatorToPosition;

// intake/indexer
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.intake_indexer.RunIndexer;
import frc.robot.commands.intake_indexer.RunIntake;
// import frc.robot.commands.intake_indexer.RunIntake;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexer;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexerJoystick;

// pivot imports
import frc.robot.subsystems.Pivot;
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.pivot.PIDForPivot;


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
  private final ArmToPos m_armToPos = new ArmToPos(m_arm, 0.781); 
  private JoystickButton m_armToPosButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmButtonID);
  // use these for actual code
  private final JoystickButton m_armIntakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeIntakeButtonID);
  private final JoystickButton m_armOuttakeButton = new JoystickButton(m_secondJoystick, ArmConstants.kArmOuttakeOuttakeButtonID);
  */


  // see https://docs.google.com/spreadsheets/d/1hX9_6sB4cpDO8FewZYjP8up_QC9e0-G85cX7ijPXfBs/ for google sheet constants
  private final ArmToPos m_armToL1 = new ArmToPos(m_arm, ArmConstants.kL1ArmTickPosition);
  private final ArmToPos m_armToL2 = new ArmToPos(m_arm, ArmConstants.kL2ArmTickPosition);
  private final ArmToPos m_armToL3 = new ArmToPos(m_arm, ArmConstants.kL3ArmTickPosition);
  private final ArmToPos m_armToL4 = new ArmToPos(m_arm, ArmConstants.kL4ArmTickPosition);

  private final ArmToPos m_armToSubstationIntake = new ArmToPos(m_arm, ArmConstants.kSubstationTickPosition);
  private final ArmToPos m_armToGroundIntake = new ArmToPos(m_arm, ArmConstants.kGroundIntakeTickPosition); // probably used in different command
  private final ArmToPos m_armToGroundIntake1 = new ArmToPos(m_arm, ArmConstants.kGroundIntakeTickPosition); // probably used in different command

  private final JoystickButton m_stowButton = new JoystickButton(m_secondJoystick, IOConstants.kStowButtonID);
  private final JoystickButton m_substationIntakeButton = new JoystickButton(m_secondJoystick, IOConstants.kSubstationButtonID);


  private final EndEffector m_endEffector = new EndEffector();
  private SpinEndEffectorMotor m_intakeEndEffector = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_intakeEndEffector1 = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_outtakeEndEffector = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
  private SpinEndEffectorMotor m_outtakeEndEffector1 = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
  private SpinEndEffectorMotor m_outtakeEndEffectorAuto = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
   

  private JoystickButton m_spinEndEffectorButton = new JoystickButton(m_driverJoystick, IOConstants.kEndEffectorOuttakeButtonID);

  private SpinEndEffectorMotor m_joystickOverrideSpinEndEffector = new SpinEndEffectorMotor(m_endEffector, 0, true, m_secondJoystick); 
  // lowkey terrible coding practice
  // note that the speed here doesn't matter so I'm not going to fret about that too much




  private Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);

  // Input of 15 means that the elevator will ideally move up by 15 inches. This was just chosen as a test.
  // this is a trapezoidal command!!
  // private JoystickButton m_elevatorTestButton = new JoystickButton(m_secondJoystick, ElevatorConstants.kElevatorTestButtonID);
  // private ElevatorToPosition m_elevatorTest = new ElevatorToPosition(m_elevator, 15);
  private JoystickButton m_elevatorResetButton = new JoystickButton(m_driverJoystick, IOConstants.kElevatorResetButtonID);

  private ElevatorToPosition m_elevatorToL2 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL2ElevatorHeight);
  private ElevatorToPosition m_elevatorToL3 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL3ElevatorHeight);
  private ElevatorToPosition m_elevatorToL4 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL4ElevatorHeight);

  private ElevatorToPosition m_elevatorToSubstationIntake = new ElevatorToPosition(m_elevator, ElevatorConstants.kSubstationIntakeElevatorHeight);
  private ElevatorToPosition m_elevatorToGround = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround2 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround3 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround4 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position




  

  private Pivot m_pivot = new Pivot();

  // we actually only have 2 pivot positions -- the intake from the ground, and the stow upwards
  // intake from the ground is at approximately 0.1385 rotations
  // pivot stow is at approximately 0.8069 rotations

  private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_secondJoystick);

  private PIDForPivot m_pivotPIDToStow = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  private PIDForPivot m_pivotPIDToIntake = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  // private JoystickButton m_pivotButtonToStow = new JoystickButton(m_secondJoystick, PivotConstants.kPivotStowButtonID);
  // private JoystickButton m_pivotButtonToIntake = new JoystickButton(m_secondJoystick, PivotConstants.kPivotIntakeButtonID);





  // begin intake/indexer
  private Intake m_intake = new Intake();
  private final double kIntakeIndexerSpeed = 0.6;
  // private RunIntake m_intakeIn = new RunIntake(m_intake, kIntakeIndexerSpeed); // positive speed == intake in
  // private RunIntake m_intakeOut = new RunIntake(m_intake, -kIntakeIndexerSpeed);

  private Indexer m_indexer = new Indexer();
  private RunIndexer m_indexerIn = new RunIndexer(m_indexer, kIntakeIndexerSpeed);
  private RunIndexer m_indexerOut = new RunIndexer(m_indexer, -kIntakeIndexerSpeed);

  // Same speed:
  // private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, kIntakeIndexerSpeed);
  // Different speed:
  // private RunIntakeWithIndexer m_intakeWithIndexer = new RunIntakeWithIndexer(m_intake, m_indexer, 0.3, 0.5);

  private RunIntakeWithIndexer m_spinIntakeIndexerRollers = new RunIntakeWithIndexer(m_intake, m_indexer, kIntakeIndexerSpeed); // used for ground intake coral
  private RunIntakeWithIndexer m_outtakeIntakeIndexerRollers = new RunIntakeWithIndexer(m_intake, m_indexer, -kIntakeIndexerSpeed);

  // combine these into one command
  
  private RunIntakeWithIndexerJoystick m_manualSpinIndexerIntake = new RunIntakeWithIndexerJoystick(m_secondJoystick, m_intake, m_indexer);
  
  
  // note: these buttons are both not assigned and also missing the right IDs
  // private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeInButtonID);
  // private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeOutButtonID);
  // private JoystickButton m_indexerInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerInButtonID);
  // private JoystickButton m_indexerOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerOutButtonID);

  private JoystickButton m_groundIntakeCoralButton = new JoystickButton(m_driverJoystick, IOConstants.kGroundIntakeCoralButtonID);
  // end intake/indexer
  // private final MoveForTime m_moveForTime = new MoveForTime(m_swerve, 7, -0.5, 0, 0);
  // private DriveForwardAndPlace m_driveForwardAndPlace = new DriveForwardAndPlace(m_armToL1, m_elevatorToGround, m_moveForTime, m_outtakeEndEffectorAuto);

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

  private JoystickButton m_testButton = new JoystickButton(m_driverJoystick, 1);
  private JoystickButton m_test2Button = new JoystickButton(m_driverJoystick, 2);
  private void configureBindings() {
    // see discord channel for button bindings

    // this is the testing section.
    // everything here should only be testing thing


    m_testButton.whileTrue(m_outtakeEndEffector);
    m_test2Button.whileTrue(m_intakeEndEffector);
    
    m_elevatorResetButton.whileTrue(m_elevator.resetElevatorEncoder());

    // note:
    // if a line is commented out using "/////" (5 in a row), that is for testing purposes (and has not been tested)
    // if it has 6 in a row, that means the thing has been tested and works.

///////////////////////////////////////////////////////////
    // driver joystick bindings:
    ///// m_resetHeadingButton.onTrue(m_resetHeadingCommand); // button 4, y button
    // to add - button 5 - left align to reef (vision)
    // to add - button 6 - right align to reef (vision)
    ////// 
    // m_spinEndEffectorButton.whileTrue(m_outtakeEndEffector); // button 7, spin arm outtake roller



    // button 8 -- ground intake coral
    // to my knowledge, this button, when pressed, is supposed to 
    // 1. make the pivot go down, arm in pickup position, and move the elevator down as well
    // 2. indefinitely spin the intake/indexer wheels + the end effector wheels

    // when released, this command is to move the pivot up, and stop spinning everything.
    // NOTE this command (as of now) DOES NOT move the arm to the desired location to ground intake coral

    // --> this might actually be a good idea if arm is manually moved
  
    // lowkey unsure if this code works


    // the button that drops the intake and spins the roller
    // when it goes down, everything is spinning inwards (intake, indexer, end effector)
    // then
    // when the button is released
    // intake will go back up
    // intake, indexer, and outtake will continue to spin for either 2 seconds or until beam break is triggered
    // then after,
    // the intake and the indexer will outtake for 2 seconds

    
    // the .until is kept here as when the elevator/arm reach "ground", there is no ff needed to hold them up -- they are at the lowest position
    // ok i have been convinced by kaitlyn
    // turns out the .until stops the elevator from going down

    //////
    
    // m_groundIntakeCoralButton.whileTrue(
    //   Commands.parallel(
    //     m_elevatorToGround4, 
    //     m_armToGroundIntake1,
    //     m_pivotPIDToIntake, 
    //     m_spinIntakeIndexerRollers, 
    //     m_intakeEndEffector1
    //   )
    // );
    
    
    


    // what it does is move the pivot up (only)
    ////// (works!)
    // m_groundIntakeCoralButton.onFalse(
    //   Commands.parallel(
    //     m_pivotPIDToStow,
    //     m_intakeEndEffector.withTimeout(2),
    //     Commands.sequence( // todo: put a beam break thing
    //       m_spinIntakeIndexerRollers.withTimeout(2), m_outtakeIntakeIndexerRollers.withTimeout(2)
    //     )
    //   )
    // );
  
    

    // end driver joystick bindings

/////////////////////////////////////////////////////////////

    // start operator joystick bindings
    // for the joystick axes, once buttons 7/8 are pressed, manual arm control happens
    // same with pivot/elevator override

    // NOTE: I may need to fix these commands (and/or other commands) so that the manual override can interrupt other commands
    // button id 7 (back left button), allows manual control of elevator/arm
    // also allows for manual intake/outtake

    // what i wrote last night was bad
    // i have rewritten it

    /*
    /////
    m_elevatorArmManualControlButton.whileTrue(
      Commands.parallel(
        m_arcadeElevator,
        m_arcadeArm,
        m_manualSpinIndexerIntake, m_joystickOverrideSpinEndEffector // these two commands go together
      )
    );
    */


    // button id 8 (back right button) allows manual control of arm/pivot
    /* /////
    m_pivotArmManualControlButton.whileTrue(
      Commands.parallel(
        m_arcadePivot,
        m_arcadeArm,
        m_manualSpinIndexerIntake,
        m_joystickOverrideSpinEndEffector
      )
    );
    */



    // L1-4 scoring
    // gonna assume it takes <1.5s to score that piece
    // finally, reset position by going back to ArmToGroundIntake position

    // note: elevator should always be reset for correctness

    // NOTE: i do not use the .until here (or later) since the feedforward must be used to keep the elevator at the reasonable position
    
    
    //////
    
    m_L1ScoringButton.onTrue(
      Commands.parallel(
        // m_armToL1.until(m_armToL1::atSetpoint),
        m_armToL1,
        m_elevatorToGround3
      )
    );
    
    


    //************************************************************** */
    // MASSIVE TODO - make sure that this works and doesnt break metal
    //************************************************************** */
    // it works!

    // note: may need to continue arm ff for later
    // will also need to redo commands
    // btw arm to position might never end


    // -- L2, L3, L4 works.
    //////
    m_L2ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL2.until(m_elevatorToL2::atSetpoint),
        // m_armToL2.until(m_armToL2::atSetpoint)
        m_elevatorToL2,
        Commands.sequence(new WaitCommand(0.2), m_armToL2)
      )
    );
    

    // L3
    //////
    m_L3ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL3.until(m_elevatorToL3::atSetpoint),
        // m_armToL3.until(m_armToL3::atSetpoint)
        m_elevatorToL3,
        Commands.sequence(new WaitCommand(0.2), m_armToL3)
      )
    );
    

    // L4
    //////
    m_L4ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL4.until(m_elevatorToL4::atSetpoint),
        // m_armToL4.until(m_armToL4::atSetpoint)
        m_elevatorToL4,
        Commands.sequence(new WaitCommand(0.2), m_armToL4)
      )
    );
    


    // bind buttons 5, 6
    //////
    m_stowButton.onTrue(Commands.parallel(
      m_armToGroundIntake,
      m_elevatorToGround2
    )); // button 5 -- stow arm and elevator

    
    
     

    // button 6 -- move arm to substation intake position
    // will fix when beam break is added, but for now, this is when this thing is held.

    // the at setpoint is to move the command along
    /////
    
    m_substationIntakeButton.onTrue(
      Commands.parallel(
        m_elevatorToSubstationIntake,
        Commands.sequence(new WaitCommand(0.5), m_armToSubstationIntake),
        // m_spinEndEffector
        Commands.sequence(new WaitCommand(0.2), m_intakeEndEffector)) // note: this command can maybe spin indefinitely
    ); 
    
    
  }




  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    // return m_driveForwardAndPlace;
  }



  private void bindSubsystemCommands() {
    ////// 
    // m_swerve.setDefaultCommand(m_swerveJoystick);


    // note: there are really no default commands 
    // instead, the code should do everything for us
    // when a manual override is needed, a button is already pressed
    // thus, see the button binding on true part for that
    // (note to self): I may get a watchdog error for this in which case I'll bind null or something

    m_arm.setDefaultCommand(m_arcadeArm);
    // m_pivot.setDefaultCommand(m_arcadePivot);
    m_elevator.setDefaultCommand(m_arcadeElevator);
    // m_elevator.setDefaultCommand(m_elevatorPosition);
  }
}
  
