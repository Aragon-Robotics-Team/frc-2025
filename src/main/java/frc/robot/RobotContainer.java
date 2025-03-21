// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.jar.Attributes.Name;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.SwerveJoystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
import frc.robot.commands.auto.DriveForwardL4;
import frc.robot.commands.auto.MoveForTime;

import frc.robot.subsystems.EndEffector;

// elevator imports
import frc.robot.subsystems.Elevator;
import frc.robot.commands.elevator.ArcadeElevator;
import frc.robot.commands.elevator.ElevatorDealgae;
import frc.robot.commands.elevator.ElevatorToPosition;

// intake/indexer
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.intake_indexer.RunIndexer;
import frc.robot.commands.intake_indexer.RunIntake;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexer;
import frc.robot.commands.intake_indexer.RunIntakeWithIndexerJoystick;

// pivot imports
import frc.robot.subsystems.Pivot;
import frc.robot.commands.pivot.ArcadePivot;
import frc.robot.commands.pivot.PIDForPivot;

import frc.robot.subsystems.Climb;
import frc.robot.commands.climb.JoystickServo;
import frc.robot.commands.climb.ServoMovement;
import frc.robot.commands.climb.SpinVortexRotations;

@SuppressWarnings("unused") // thanks 254

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than commandsthe scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
 
  // The robot's subsystems and  are defined here...

  private final Joystick m_driverJoystick = new Joystick(0);
  private final Joystick m_secondJoystick = new Joystick(1);



  private final JoystickButton m_resetHeadingButton = new JoystickButton(m_driverJoystick, IOConstants.kResetHeadingButtonID);


  private final JoystickButton m_elevatorArmManualControlButton = new JoystickButton(m_secondJoystick, IOConstants.kElevatorArmManualOverrideButtonID); // 7
  private final JoystickButton m_pivotRollerManualControlButton = new JoystickButton(m_secondJoystick, IOConstants.kPivotArmManualOverrideButtonID); // 8 (i think)





  private final JoystickButton m_L1ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL1ScoringButtonID);
  private final JoystickButton m_L2ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL2ScoringButtonID);
  private final JoystickButton m_L3ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL3ScoringButtonID);
  private final JoystickButton m_L4ScoringButton = new JoystickButton(m_secondJoystick, IOConstants.kL4ScoringButtonID);

  // private final JoystickButton m_L2DealgaeButton = new JoystickButton(m_driverJoystick, IOConstants.kL2DealgaeButtonID);
  // private final JoystickButton m_L3DealgaeButton = new JoystickButton(m_driverJoystick, IOConstants.kL3DealgaeButtonID);

  // private final POVButton m_dealgaeButton = new POVButton(m_driverJoystick, 0);

  private final POVButton m_L2DealgaeButton = new POVButton(m_secondJoystick, 270); // left d-pad
  private final POVButton m_L3DealgaeButton = new POVButton(m_secondJoystick, 90); // right d-pad

  private final Arm m_arm = new Arm(); 
  private final ArcadeArm m_arcadeArm = new ArcadeArm(m_arm, m_secondJoystick);
  private final ArcadeArm m_arcadeArm1 = new ArcadeArm(m_arm, m_secondJoystick);
  private final ArcadeArm m_arcadeArm2 = new ArcadeArm(m_arm, m_secondJoystick);


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
  private final ArmToPos m_armToL1v2 = new ArmToPos(m_arm, ArmConstants.kL1ArmTickPosition);
  private final ArmToPos m_armToL2 = new ArmToPos(m_arm, ArmConstants.kL2ArmTickPosition);
  private final ArmToPos m_armToL3 = new ArmToPos(m_arm, ArmConstants.kL3ArmTickPosition);
  private final ArmToPos m_armToL4 = new ArmToPos(m_arm, ArmConstants.kL4ArmTickPosition);

  private final ArmToPos m_armToL2Dealgae = new ArmToPos(m_arm, ArmConstants.kL2DealgaeArmTickPosition);
  private final ArmToPos m_armToL3Dealgae = new ArmToPos(m_arm, ArmConstants.kL3DealgaeArmTickPosition);

  private final ArmToPos m_armToSubstationIntake = new ArmToPos(m_arm, ArmConstants.kSubstationTickPosition);
  private final ArmToPos m_armToGroundIntake = new ArmToPos(m_arm, ArmConstants.kGroundIntakeTickPosition); // probably used in different command
  private final ArmToPos m_armToGroundIntake1 = new ArmToPos(m_arm, ArmConstants.kGroundIntakeTickPosition); // probably used in different command

  private final JoystickButton m_stowButton = new JoystickButton(m_secondJoystick, IOConstants.kStowButtonID);


  private final EndEffector m_endEffector = new EndEffector();
  private SpinEndEffectorMotor m_intakeEndEffector = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_intakeEndEffector1 = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_intakeEndEffector2 = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_intakeEndEffector3 = new SpinEndEffectorMotor(m_endEffector, 0.9, false, m_secondJoystick);
  private SpinEndEffectorMotor m_outtakeEndEffector = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
  private SpinEndEffectorMotor m_outtakeEndEffector1 = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
  private SpinEndEffectorMotor m_outtakeEndEffectorAuto = new SpinEndEffectorMotor(m_endEffector, -0.9, false, m_secondJoystick); // spin out is probably a negative speed, and this just spins it out
   

  private JoystickButton m_spinEndEffectorButton = new JoystickButton(m_driverJoystick, IOConstants.kEndEffectorOuttakeButtonID);
  
  private JoystickButton m_outtakeEndEffectorButton = new JoystickButton(m_driverJoystick, IOConstants.kEndEffectorOuttakeButtonID);
  private JoystickButton m_intakeEndEffectorButton = new JoystickButton(m_driverJoystick, IOConstants.kEndEffectorIntakeButtonID);
 
  private SpinEndEffectorMotor m_joystickOverrideSpinEndEffector = new SpinEndEffectorMotor(m_endEffector, 0, true, m_secondJoystick); 
  private SpinEndEffectorMotor m_joystickOverrideSpinEndEffector1 = new SpinEndEffectorMotor(m_endEffector, 0, true, m_secondJoystick); 
  // lowkey terrible coding practice
  // note that the speed here doesn't matter so I'm not going to fret about that too much




  public Elevator m_elevator = new Elevator();
  private ArcadeElevator m_arcadeElevator = new ArcadeElevator(m_secondJoystick, m_elevator);
  private ArcadeElevator m_arcadeElevator1 = new ArcadeElevator(m_secondJoystick, m_elevator);

  // Input of 15 means that the elevator will ideally move up by 15 inches. This was just chosen as a test.
  // this is a trapezoidal command!!
  // private JoystickButton m_elevatorTestButton = new JoystickButton(m_secondJoystick, ElevatorConstants.kElevatorTestButtonID);
  // private ElevatorToPosition m_elevatorTest = new ElevatorToPosition(m_elevator, 15);
  // private JoystickButton m_elevatorResetButton = new JoystickButton(m_driverJoystick, IOConstants.kElevatorResetButtonID);
  private POVButton m_elevatorResetButton = new POVButton(m_secondJoystick, 180);

  private ElevatorToPosition m_elevatorToL2 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL2ElevatorHeight);
  private ElevatorToPosition m_elevatorToL3 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL3ElevatorHeight);
  private ElevatorToPosition m_elevatorToL4 = new ElevatorToPosition(m_elevator, ElevatorConstants.kL4ElevatorHeight);

  private ElevatorToPosition m_elevatorToL2Dealgae = new ElevatorToPosition(m_elevator, ElevatorConstants.kL2DealgaeElevatorHeight);
  private ElevatorToPosition m_elevatorToL3Dealgae = new ElevatorToPosition(m_elevator, ElevatorConstants.kL3DealgaeElevatorHeight);
  // private ElevatorDealgae m_elevatorDealgae = new ElevatorDealgae(m_elevator);

  private ElevatorToPosition m_elevatorToGround = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround2 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround3 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position
  private ElevatorToPosition m_elevatorToGround4 = new ElevatorToPosition(m_elevator, 0.01); // reset elevator position




  
  // chat our spark max fell off
  // private Pivot m_pivot = new Pivot();

  // we actually only have 2 pivot positions -- the intake from the ground, and the stow upwards
  // intake from the ground is at approximately 0.1385 rotations
  // pivot stow is at approximately 0.8069 rotations

  // private ArcadePivot m_arcadePivot = new ArcadePivot(m_pivot, m_secondJoystick);

  // private PIDForPivot m_pivotPIDToStow = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  // private PIDForPivot m_pivotPIDToStow1 = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  // private PIDForPivot m_pivotPIDToStow2 = new PIDForPivot(m_pivot, PivotConstants.kPivotStowPosition);
  // private PIDForPivot m_pivotPIDToIntake = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  // private PIDForPivot m_pivotPIDToIntake2 = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  // private PIDForPivot m_pivotPIDToIntake3 = new PIDForPivot(m_pivot, PivotConstants.kPivotIntakePosition);
  // private JoystickButton m_pivotButtonToStow = new JoystickButton(m_secondJoystick, PivotConstants.kPivotStowButtonID);
  // private JoystickButton m_pivotButtonToIntake = new JoystickButton(m_secondJoystick, PivotConstants.kPivotIntakeButtonID);





  // begin intake/indexer
  private Intake m_intake = new Intake();
  private final double kIntakeIndexerSpeed = 0.9;
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
  private RunIntakeWithIndexer m_spinIntakeIndexerRollers1 = new RunIntakeWithIndexer(m_intake, m_indexer, kIntakeIndexerSpeed); // used for ground intake coral
  private RunIntakeWithIndexer m_outtakeIntakeIndexerRollers1 = new RunIntakeWithIndexer(m_intake, m_indexer, -kIntakeIndexerSpeed);

  // combine these into one command
  
  private RunIntakeWithIndexerJoystick m_manualSpinIndexerIntake = new RunIntakeWithIndexerJoystick(m_secondJoystick, m_intake, m_indexer);
  private RunIntakeWithIndexerJoystick m_manualSpinIndexerIntake1 = new RunIntakeWithIndexerJoystick(m_secondJoystick, m_intake, m_indexer);
  
  
  // note: these buttons are both not assigned and also missing the right IDs
  // private JoystickButton m_intakeInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeInButtonID);
  // private JoystickButton m_intakeOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIntakeOutButtonID);
  // private JoystickButton m_indexerInButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerInButtonID);
  // private JoystickButton m_indexerOutButton = new JoystickButton(m_secondJoystick, IntakeConstants.kIndexerOutButtonID);

  private JoystickButton m_groundIntakeCoralButton = new JoystickButton(m_secondJoystick, IOConstants.kGroundIntakeCoralButtonID);
  // end intake/indexer
  //private MoveForTime m_leaveAuto = new MoveForTime(m_swerve, 4, 0, -0.6, 0);
  //private DriveForwardL4 m_driveForwardL4 = new DriveForwardL4(m_swerve, m_arm, m_elevator, m_endEffector, m_secondJoystick);




  private Climb m_climb = new Climb();
  // check these speeds and rotations
  private SpinVortexRotations m_getCage = new SpinVortexRotations(m_climb, 0.8, 55.692);
  private SpinVortexRotations m_retractCage = new SpinVortexRotations(m_climb, -0.95, 0.5); // move motor back to 5 rotations

  private ServoMovement m_lockServo = new ServoMovement(m_climb, 0.59833333, false);
  private ServoMovement m_getCageServo = new ServoMovement(m_climb, 0.24666666666, true); // move from 0.0 to 1.0???
  private ServoMovement m_retractCageServo = new ServoMovement(m_climb, 0.5983333, false); // move back
  


  private final POVButton m_climbButton = new POVButton(m_secondJoystick, 0); // up d-pad button

  // see below (bindings) for the actual command being run
  // private RunCommand m_climbCommand = new RunCommand(
  //   () -> (new WaitUntilCommand(() -> !m_climbButton.getAsBoolean()))
  //   .andThen(() -> m_getCage.schedule())
  //   .until(() -> m_climbButton.getAsBoolean())
  //   .andThen(() -> m_moveServo.schedule())
  //   .andThen(() -> m_retractCage.schedule()), 
  //   m_climb
  // );

  

  // servo position testing
  
  // private Joystick testJoystick = new Joystick(2);
  // private JoystickServo m_moveServoWithJoystick = new JoystickServo(testJoystick, m_climb);

  public final SwerveDrive m_swerve = new SwerveDrive();
  public final SwerveJoystick m_swerveJoystick = new SwerveJoystick(m_swerve, m_driverJoystick);
  private final InstantCommand m_resetHeadingCommand = m_swerve.resetHeadingCommand();
  
  private SendableChooser<Command> m_autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("L2 Elevator", m_elevatorToL2);
    NamedCommands.registerCommand("L3 Elevator", m_elevatorToL3);
    NamedCommands.registerCommand("L4 Elevator", m_elevatorToL4);
    NamedCommands.registerCommand("L1 Arm", m_armToL1);
    NamedCommands.registerCommand("L2 Arm", m_armToL2);
    NamedCommands.registerCommand("L3 Arm", m_armToL3);
    NamedCommands.registerCommand("L4 Arm", m_armToL4);
    NamedCommands.registerCommand("Substation Arm", m_armToSubstationIntake);
    NamedCommands.registerCommand("Ground Arm", m_armToGroundIntake);
    NamedCommands.registerCommand("Spin End Effector", m_intakeEndEffector);
    NamedCommands.registerCommand("Outtake End Effector", m_outtakeEndEffectorAuto);
    NamedCommands.registerCommand("In Indexer", m_indexerIn);
    NamedCommands.registerCommand("Out Indexer", m_indexerOut);
    NamedCommands.registerCommand("Intake w/ Indexer", m_spinIntakeIndexerRollers);
    NamedCommands.registerCommand("Outtake w/ Indexer", m_outtakeIntakeIndexerRollers);

    m_autoChooser = AutoBuilder.buildAutoChooser();
    System.out.println("build auto chooser");
    //m_autoChooser.setDefaultOption("Drive, L4", m_driveForwardL4);
    //m_autoChooser.addOption("Move Auto", m_leaveAuto);
    
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
    // SmartDashboard.putData("Reset_Heading", m_swerve.resetHeadingCommand());
    // Configure the trigger bindings
    bindSubsystemCommands();
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

    // this is the testing section.
    // everything here should only be testing thing


    m_outtakeEndEffectorButton.whileTrue(m_outtakeEndEffector);
    m_intakeEndEffectorButton.whileTrue(m_intakeEndEffector);
    
    // m_elevatorResetButton.whileTrue(m_elevator.resetElevatorEncoder());

    // note:
    // if a line is commented out using "/////" (5 in a row), that is for testing purposes (and has not been tested)
    // if it has 6 in a row, that means the thing has been tested and works.

///////////////////////////////////////////////////////////
    // driver joystick bindings:

    // swerve joystick binded below
    m_resetHeadingButton.whileTrue(m_resetHeadingCommand); // button 4, y button
    // to add - button 5 - left align to reef (vision)
    // to add - button 6 - right align to reef (vision)
    m_spinEndEffectorButton.whileTrue(m_outtakeEndEffector); // button 5, spin arm outtake roller


  

    
  
    

    // end driver joystick bindings

/////////////////////////////////////////////////////////////

// 02/28 -- reprogramming George's joystick


    // climb command
    m_climbButton.onTrue(
      Commands.sequence(
        new WaitCommand(0.1), // would be here to unpress but there's another wait command down there
        Commands.parallel(
          m_armToL1v2,
          Commands.sequence( new WaitCommand(0.3),
            Commands.deadline(new WaitCommand(0.3), m_getCageServo),
            Commands.sequence( new WaitCommand(0.5), m_getCage)
          ) // need to wait (0.5s) to make sure the arm is mostly out of the way
          // Commands.sequence( new WaitCommand(0.5), m_getCage) // need to wait (0.5s) to make sure the arm is mostly out of the way
          ).until(() -> m_climbButton.getAsBoolean()),

        Commands.deadline(new WaitCommand(0.3), m_retractCageServo),
        m_retractCage
      )
    );

    // button 7 -- the middle button
    m_elevatorArmManualControlButton.onTrue(
      Commands.parallel(
        m_arcadeElevator1,
        m_arcadeArm1
      )
    );
    


    // button id 8 allows manual roller + pivot to move 
    // changed from arm/intake rollers
    m_pivotRollerManualControlButton.onTrue(
      Commands.parallel(
        // m_arcadePivot,
        m_manualSpinIndexerIntake1
      )
    );
    


    // need to redo these -- dpad commands
    // use povUp to do the thing

    m_L2DealgaeButton.onTrue(
      Commands.parallel(
        m_armToL2Dealgae,
        m_elevatorToL2Dealgae
      )
    );
    
    m_L3DealgaeButton.onTrue(
      Commands.parallel(
        m_armToL3Dealgae,
        m_elevatorToL3Dealgae
      )
    );


    
    // m_dealgaeButton.onTrue(
    //   Commands.parallel(
    //   m_armToL2Dealgae,
      // m_elevatorDealgae)
    // );

    // m_dealgaeButton.onTrue(
    //   Commands.parallel(
    //     m_armToL2Dealgae,
    //     () -> {
    //       if (Math.abs(m_elevator.getElevatorPosition() - ElevatorConstants.kL3ElevatorHeight) <= ElevatorConstants.kDealgaeThreshold) {
    //         return m_elevatorToL3Dealgae;
    //       } else if (Math.abs(m_elevator.getElevatorPosition() - ElevatorConstants.kL2ElevatorHeight) <= ElevatorConstants.kDealgaeThreshold) {
    //         return m_elevatorToL2Dealgae;
    //       }
    //     }.get();
    //   )
    // )
    


    // L1-4 scoring
    // gonna assume it takes <1.5s to score that piece
    // finally, reset position by going back to ArmToGroundIntake position

    // note: elevator should always be reset for correctness
    
    m_L1ScoringButton.onTrue(
      Commands.parallel(
        // m_armToL1.until(m_armToL1::atSetpoint),
        m_armToL1,
        m_elevatorToGround3
      )
    );
    
    m_L2ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL2.until(m_elevatorToL2::atSetpoint),
        // m_armToL2.until(m_armToL2::atSetpoint)
        m_elevatorToL2,
        Commands.sequence(new WaitCommand(0.2), m_armToL2)
      )
    );
    

    // L3
    // add pivot pid to balance out the cog
    m_L3ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL3.until(m_elevatorToL3::atSetpoint),
        // m_armToL3.until(m_armToL3::atSetpoint)
        m_elevatorToL3,
        Commands.sequence(new WaitCommand(0.2), m_armToL3)
        // m_pivotPIDToIntake2
      )
    );

    // L4
    // also move the pivot down
    m_L4ScoringButton.onTrue(
      Commands.parallel(
        // m_elevatorToL4.until(m_elevatorToL4::atSetpoint),
        // m_armToL4.until(m_armToL4::atSetpoint)
        m_elevatorToL4,
        Commands.sequence(new WaitCommand(0.2), m_armToL4)
        // m_pivotPIDToIntake3
      )
    );
    


    // bind button 5 (left trigger)
    // move pivot up for simplicity
    m_stowButton.onTrue(Commands.parallel(
      m_armToGroundIntake,
      m_elevatorToGround2
      // m_pivotPIDToStow2
    )); // button 5 -- stow arm and elevator

    
    // button 6 -- do the intake thing

    
    m_groundIntakeCoralButton.whileTrue(
      Commands.parallel(
        m_elevatorToGround4, 
        m_armToGroundIntake1,
        // m_pivotPIDToIntake, 
        m_spinIntakeIndexerRollers, 
        m_intakeEndEffector1
      )
    );

    // what it does is move the pivot up (only)
    m_groundIntakeCoralButton.onFalse(
      Commands.parallel(
        // m_pivotPIDToStow1,
        m_intakeEndEffector3.withTimeout(1),
        Commands.sequence( // todo: put a beam break thing
          m_spinIntakeIndexerRollers1.withTimeout(1), m_outtakeIntakeIndexerRollers1.withTimeout(1)
        )
      )
    );
    
    
  }




  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
    // return m_driveForwardAndPlace;
  }



  private void bindSubsystemCommands() {
    ////// 
    m_swerve.setDefaultCommand(m_swerveJoystick);
    m_lockServo.schedule();
    // m_pivot.setDefaultCommand(m_arcadePivot);
    // m_climb.setDefaultCommand(m_moveServoWithJoystick);
  }
}
  
