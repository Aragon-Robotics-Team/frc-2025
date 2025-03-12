// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.SwerveDrive;

public class SwerveJoystick extends Command {
  /** Creates a new SwerveJoystick. */

  private final Joystick m_joystick;
  private final SlewRateLimiter m_xSlewRateLimiter;
  private final SlewRateLimiter m_ySlewRateLimiter;
  private final SwerveDrive m_swerveDrive;
  public int m_driveMode = 0;

  public SwerveJoystick(SwerveDrive swerveDrive, Joystick joystick) {
    
    m_xSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_ySlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_joystick = joystick;
    m_swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {System.out.println("SwerveJoystick Initialized");}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = m_joystick.getRawAxis(IOConstants.kJoystickYAxis);
    double ySpeed = m_joystick.getRawAxis(IOConstants.kJoystickXAxis);
    double turningSpeed = m_joystick.getRawAxis(IOConstants.kJoystickRotAxis);


    // if (m_driveMode == 0) { //standard mode;
    // //Makes the speed response exponential in relation to the joystick input.
    // //That way, the first little bit of joystick input gives more control.  
    // xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -1;
    // ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -1;
    // turningSpeed = Math.signum(turningSpeed) * (Math.pow(2, Math.abs(turningSpeed)) -1) * -1;
    // //xSpeed = Math.pow(xSpeed, 5);
    // // ySpeed = Math.pow(ySpeed, 5);
    // // turningSpeed = Math.pow(turningSpeed, 5);
    // }
    // else if(m_driveMode == 1)
    // {
    //   xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -0.1;
    //   ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -0.1;
    //   turningSpeed = Math.signum(turningSpeed) * (Math.pow(2, Math.abs(turningSpeed)) -1) * -0.1;
    // }


      // xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -1;
      // ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -1;
      // turningSpeed = Math.signum(turningSpeed) * (Math.pow(2, Math.abs(turningSpeed)) -1) * -1;

      xSpeed = Math.signum(xSpeed) * Math.pow(Math.abs(xSpeed), 1.75);
      ySpeed = Math.signum(ySpeed) * Math.pow(Math.abs(ySpeed), 1.75);
      // turningSpeed = Math.pow(turningSpeed, 5);
    
    

    // SmartDashboard.putNumber("Joystick/xSpeedRaw", xSpeed);
    // SmartDashboard.putNumber("Joystick/ySpeedRaw", ySpeed);
    // SmartDashboard.putNumber("Joystick/turningSpeedRaw", turningSpeed);

    //apply deadband
    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > IOConstants.kDeadband ? turningSpeed : 0.0;

    //use SlewRateLimiter with DriveConstants
    xSpeed = m_xSlewRateLimiter.calculate(xSpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    ySpeed = m_ySlewRateLimiter.calculate(ySpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    turningSpeed = turningSpeed * DriveConstants.kMaxTurningRadiansPerSecond;
    SmartDashboard.putNumber("Joystick/xSpeedCommanded", xSpeed);
    SmartDashboard.putNumber("Joystick/ySpeedCommanded", ySpeed);
    SmartDashboard.putNumber("Joystick/turningSpeedCommanded", turningSpeed);
    //Logger.recordOutput(getName(), desiredSwerveModuleStates);

    m_swerveDrive.driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,turningSpeed, m_swerveDrive.getAngle()));
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