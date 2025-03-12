// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
// aka controller constants
public final class IOConstants {
    public static final double kDeadband = 0.05;
    
    public static final int kDriveJoystickId = 0;

    
    public static int kJoystickXAxis = 0; // Axis Zero is the Left-Right axis on the left stick of the joystick. It is labeled as the X-axis on driverstation. On driverstation, left gives a negative input, whilst right gives a positive input. On the field, moving this joystick left gives a positive Y field direction whilst moving the joystick right gives a negative Y field direction
    public static int kJoystickYAxis = 1; // Axis One is the Up-Down axis on the left stick of the joystick. It is labeled ad the Y-axis on driverstation. On driverstation, up gives a negative input, whilst down gives a positive input. On the field, moving this joystick up gives a positive X field direction whilst moving the joystick down gives a negative X field direction
    public static int kJoystickRotAxis = 4;

    // note: these are button constants for the joystick for the robot driver (port 0)
    public static final int kEndEffectorOuttakeButtonID = 5;
    public static final int kEndEffectorIntakeButtonID = 6; // right trigger
    // public static final int kL2DealgaeButtonID = 3;
    // public static final int kL3DealgaeButtonID = 4;
    public static final int kResetHeadingButtonID = 4; // y button
    public static final int kVisionLeftAlignButtonID = 7; // top left button
    public static final int kVisionRightAlignButtonID = 8; // top right button 
    public static final int kVisionSnapToAngleButtonID = 1;   
    public static final int kElevatorResetButtonID = 3;
    
    // this is for both indexer/intake mechanism outtake (spinning out)
    // public static final int kEndEffectorOuttakeButtonID = 7;

    // this is for ground intaking coral (see exact specs)
    

    // button constants for robot operator:
    public static final int kElevatorArmManualOverrideButtonID = 7;
    public static final int kPivotArmManualOverrideButtonID = 8;


    public static final int kGroundIntakeCoralButtonID = 6;

    // L1-4 scoring
    public static final int kL1ScoringButtonID = 1;
    public static final int kL2ScoringButtonID = 3;
    public static final int kL3ScoringButtonID = 2;
    public static final int kL4ScoringButtonID = 4;

    // button 5: stow arm
    public static final int kStowButtonID = 5;


    // Button board
    public static final int k1stTagButtonID = 1;
    public static final int k2ndTagButtonID = 2;
    public static final int k3rdTagButtonID = 3;
    public static final int k4thTagButtonID = 4;
    public static final int k5thTagButtonID = 5;
    public static final int k6thTagButtonID = 6;
}
