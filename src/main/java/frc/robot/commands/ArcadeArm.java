// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeArm extends Command {
  
  /** Creates a new ArcadeArm. */

private final Arm m_arm;
private final Joystick m_joystick;

  public ArcadeArm(Arm arm, Joystick joystick) {
    m_arm = arm;
    m_joystick = joystick;
  
    // Use addRequirements() here to declare subsystem dependencies.
  addRequirements(m_arm);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_joystick.getRawAxis(Constants.ArmConstants.kArmYAxis) * Constants.ArmConstants.kArmMultiplier;    
    SmartDashboard.putNumber("Arm speed", speed);
    m_arm.setSpeed(speed);
    /*
    if (speed>0){
      if (m_arm.getTopLimitSwitch()){
        SmartDashboard.putNumber("Arm Speed", 0);
        m_arm.setSpeed(0);
      } else {
        SmartDashboard.putNumber("Arm Speed", speed);
        m_arm.setSpeed(speed);
      }
    } else {
      if (m_arm.getBottomLimitSwitch()){
        SmartDashboard.putNumber("Arm Speed", 0);
        m_arm.setSpeed(0);
      } else {
        SmartDashboard.putNumber("Arm Speed", speed);
        m_arm.setSpeed(speed);
      }
    }
      */
    }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
