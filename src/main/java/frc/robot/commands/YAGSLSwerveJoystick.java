// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IOConstants;
import frc.robot.subsystems.YAGSLSwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class YAGSLSwerveJoystick extends Command {
  private YAGSLSwerveDrive m_swerveDrive;
  private Joystick m_joystick;
  private final SlewRateLimiter m_xSlewRateLimiter;
  private final SlewRateLimiter m_ySlewRateLimiter;
  /** Creates a new YAGSLSwerveJoystick. */
  public YAGSLSwerveJoystick(YAGSLSwerveDrive swerveDrive, Joystick joystick) {
    m_xSlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_ySlewRateLimiter = new SlewRateLimiter(DriveConstants.kMaxTranslationalMetersPerSecond);
    m_swerveDrive = swerveDrive;
    m_joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {/*should something be here?*/}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this whole thing is basically copied
    double xSpeed = -m_joystick.getRawAxis(IOConstants.kJoystickXAxis);
    double ySpeed = m_joystick.getRawAxis(IOConstants.kJoystickYAxis);
    double turnSpeed = m_joystick.getRawAxis(IOConstants.kJoystickRotAxis);

    xSpeed = Math.signum(xSpeed) * (Math.pow(2, Math.abs(xSpeed)) -1) * -1;
    ySpeed = Math.signum(ySpeed) * (Math.pow(2, Math.abs(ySpeed)) -1) * -1;
    turnSpeed = Math.signum(turnSpeed) * (Math.pow(2, Math.abs(turnSpeed)) -1) * -1;

    xSpeed = Math.abs(xSpeed) > IOConstants.kDeadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > IOConstants.kDeadband ? ySpeed : 0.0;
    turnSpeed = Math.abs(turnSpeed) > IOConstants.kDeadband ? turnSpeed : 0.0;

    xSpeed = m_xSlewRateLimiter.calculate(xSpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    ySpeed = m_ySlewRateLimiter.calculate(ySpeed) * DriveConstants.kMaxTranslationalMetersPerSecond;
    turnSpeed = turnSpeed * DriveConstants.kMaxTurningRadiansPerSecond;

    SmartDashboard.putNumber("Joystick/xSpeed", xSpeed);
    SmartDashboard.putNumber("Joystick/ySpeed", ySpeed);
    SmartDashboard.putNumber("Joystick/turningSpeed", turnSpeed);

    //somehow set the speed and stuff here...
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerveDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
