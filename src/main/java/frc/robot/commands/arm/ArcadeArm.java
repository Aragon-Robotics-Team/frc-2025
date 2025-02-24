// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArcadeArm extends Command {
  
  /** Creates a new ArcadeArm. */

private Arm m_arm;
private Joystick m_operatorJoystick;
private double m_speed;

  public ArcadeArm(Arm arm, Joystick operatorjoystick) {
    m_arm = arm;
    m_operatorJoystick = operatorjoystick;
  
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
    m_speed = m_operatorJoystick.getRawAxis(ArmConstants.kArmManualControlAxis) * ArmConstants.kArmMultiplier;
    m_arm.setSpeed(m_speed);
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
