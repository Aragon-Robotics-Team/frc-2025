// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;

public class ArcadeElevator extends Command {  

    private Joystick m_joystick;
    private Elevator m_elevator;

    private double m_speed;
  /** Creates a new ArcadeElevator. */
  public ArcadeElevator(Joystick m_joystick2, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_joystick = m_joystick2;
    m_elevator = elevator;
  addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if we are going down the speed is negative and we are going to make our speed 0.
<<<<<<< HEAD
    m_speed = m_joystick.getRawAxis(ElevatorConstants.kElevatorYAxis);
=======
    SmartDashboard.putNumber("Elevator Joystick speed", m_joystick.getRawAxis(ElevatorConstants.kElevatorYAxis));
    if ((!(m_elevator.getLimitSwitch())))
    {
      m_elevator.setElevatorPosition(0);
    }
    if(m_joystick.getRawAxis(ElevatorConstants.kElevatorYAxis) * ElevatorConstants.kElevatorMultiplier<0 && (!(m_elevator.getLimitSwitch()))){
       m_elevator.setSpeed(0);

       SmartDashboard.putNumber("Elevator speed", 0);
>>>>>>> e2c522c (Avi/elevator position (#8))

    if (m_elevator.getLimitSwitch()) {
      m_elevator.setElevatorPosition(0);
      m_speed = 0;
    } 
    
    m_elevator.setSpeed(m_speed);
    SmartDashboard.putNumber("Elevator Joystick speed", m_speed);    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}