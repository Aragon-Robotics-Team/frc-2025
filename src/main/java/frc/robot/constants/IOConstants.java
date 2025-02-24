// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/** Add your docs here. */
// aka controller constants
public final class IOConstants {
    public static final double kDeadband = 0.05;

    public static int kJoystickXAxis = 0; // todo -- test this, but I'm pretty sure that left/right is axis 0 and thus should be x
    public static int kJoystickYAxis = 1;
    public static int kJoystickRotAxis = 4;

    // note: these are button constants for the joystick for the robot driver (port 0)
    public static final int kResetHeadingButtonID = 4; // y button
    public static final int kVisionLeftAlignButtonID = 5; // top left button
    public static final int kVisionRightAlignButtonID = 6; // top right button
    
    // this is for both indexer/intake mechanism outtake (spinning out)
    public static final int kArmOuttakeRollersButtonID = 7;

    // this is for ground intaking coral (see exact specs)
    public static final int kGroundIntakeCoralButtonID = 8;




    // button constants for robot operator:
    public static final int kElevatorArmManualOverrideButtonID = 7;
    public static final int kPivotArmManualOverrideButtonID = 8;
    
    // L1-4 scoring
    public static final int kL1ScoringButtonID = 1;
    public static final int kL2ScoringButtonID = 2;
    public static final int kL3ScoringButtonID = 3;
    public static final int kL4ScoringButtonID = 4;

}
