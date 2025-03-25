// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ArmConstants;
import frc.robot.subsystems.Arm;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmToPos extends Command {
  private double m_pos;
  private Arm m_arm;
  private double m_speed;

  private double ffc = 0.025; 
  private double ff;
  private double currentPos;
  
  // Creates a PIDController with gains kP, kI, and kD
  PIDController m_pid = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
  
  /** Creates a new ArmToPos. */
  // NOTE
  // max encoder ticks the arm should be at (near scoring position) == 0.904
  // min encoder ticks the arm should be at (at the bottom, when intaking fron the indexer) == 0.469
  public ArmToPos(Arm arm, double pos) {
    m_pos = pos;
    m_arm = arm;


    m_pid.setIZone(ArmConstants.kIZone);
    addRequirements(m_arm); 
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    currentPos = m_arm.getEncoderPosition();

    // this is assuming ticks == rotations
    // 0.727 == when torque = 0
    ff = 0;

    // logic: if we're past the 0 torque position and we're still trying to go up
    if ((m_arm.getEncoderPosition() > 0.8
    ) && (m_pos > 0.8)){
      ff = ffc*Math.sin((currentPos - 0.727)*2*Math.PI);
    }

    SmartDashboard.putNumber("Feed Forward Arm Constant", ff);

    m_speed = -m_pid.calculate(m_arm.getEncoderPosition(), m_pos) + ff;
    m_arm.setSpeed(m_speed);
    SmartDashboard.putNumber("Real Arm Position", m_arm.getEncoderPosition());
    SmartDashboard.putNumber("Arm Set Position", m_pos);
    SmartDashboard.putNumber("Arm Error", m_pos - m_arm.getEncoderPosition());
    SmartDashboard.putNumber("Arm PID Speed", -m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_arm.setSpeed(0);
  }

  public boolean atSetpoint(){
    return m_pid.atSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
    // TODO
    // NEED THE FINISH PID THING
  }
}
