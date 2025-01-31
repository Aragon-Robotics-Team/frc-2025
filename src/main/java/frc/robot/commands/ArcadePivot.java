// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadePivot extends Command {
  /** Creates a new ArcadePivot. */

  private Pivot pivot;
  private XboxController joystick;
  private Mechanism2d m_pivotSim;
  private MechanismRoot2d m_root;
  private MechanismLigament2d m_pivotObj;

  public ArcadePivot(Pivot pivot, XboxController joystick) {
    this.pivot = pivot;
    this.joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);


    // make the mech2d object thing
    m_pivotSim = new Mechanism2d(1,1);
    // place the pivot root halfway
    m_root = m_pivotSim.getRoot("pivot anchor", 0.5, 0);


    m_pivotObj = m_root.append(new MechanismLigament2d("pivot object", 0.4, 90, 6, new Color8Bit(Color.kPurple)));
    SmartDashboard.putData("Pivot 2d", m_pivotSim);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setPivotSpeed(joystick.getRawAxis(0) * PivotConstants.kArcadePivotSpeedLimiter);

    SmartDashboard.putNumber("Pivot Speed", joystick.getRawAxis(0)*PivotConstants.kArcadePivotSpeedLimiter);
    SmartDashboard.putNumber("Rotations:", pivot.getRotations()); // should be the simualted value

    m_pivotObj.setAngle(90 + pivot.getRotations()*PivotConstants.kGearRatio); // after 90 full rotations, the pivot bar will be sideways
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
