// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
// aka controller constants
public final class IOConstants {
    public static final double kDeadband = 0.05;

    public static final int kDriveJoystickId = 0;
    public static int kJoystickXAxis = 0;
    public static int kJoystickYAxis = 1;
    public static int kJoystickRotAxis = 4;

    // note: these are button constants for the joystick for the robot driver (port 0)
    public static final int kEndEffectorOuttakeButtonID = 5;
    public static final int kEndEffectorIntakeButtonID = 6; // right trigger
    public static final int kL2DealgaeButtonID = 3;
    public static final int kL3DealgaeButtonID = 4;
    public static final int kResetHeadingButtonID = 4; // y button
    public static final int kVisionLeftAlignButtonID = 5; // top left button
    public static final int kVisionRightAlignButtonID = 6; // top right button    
    public static final int kElevatorResetButtonID = 6;
    
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

}
