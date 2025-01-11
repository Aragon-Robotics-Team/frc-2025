// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.constants.*;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.units.measure.*;


public class SwerveModule extends SubsystemBase {
  private final TalonFX m_driveMotor;
  private final TalonFX m_turnMotor;
  private final DutyCycleEncoder m_turnEncoder;

  private final double m_turnEncoderOffset;
  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;

  private final String m_moduleId;

  private StatusSignal<Angle> m_drivePos;
  private StatusSignal<AngularVelocity> m_driveVel;

  public SwerveModule(int driveId, int turnId, int absoluteEncoderPort, double absoluteEncoderOffset,boolean driveReversed, boolean turningReversed, String moduleId) 
  {

    m_driveMotor = new TalonFX(driveId);
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = driveReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveSensorToMechanismRatio;
    m_driveMotor.getConfigurator().apply(driveConfig);
    m_drivingPIDController = DriveConstants.kModuleDrivePIDConstants;


    m_turnMotor = new TalonFX(turnId);
    TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.Inverted = turningReversed ? InvertedValue.Clockwise_Positive : InvertedValue.Clockwise_Positive; // CHECK THIS VALUE
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_turnMotor.getConfigurator().apply(turnConfig);
    m_turningPIDController = DriveConstants.kModuleTurnPIDConstants;
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_turnEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort)); //PWM
    m_turnEncoderOffset = absoluteEncoderOffset;
    
    m_moduleId = moduleId;
    m_drivePos = m_driveMotor.getPosition();
    m_driveVel = m_driveMotor.getVelocity();

    resetEncoders();


    SmartDashboard.putData("Swerve/Distance/reset_" + m_moduleId,  new InstantCommand(() -> resetEncoders()));
    

  }

  // in meters.
  public double getDrivePosition() {
    return m_drivePos.refresh().getValueAsDouble();
  }

  //in radians
  public double getTurningPosition() {
    return (m_turnEncoder.get() * DriveConstants.kTurnEncoderPositionToRadians) - m_turnEncoderOffset;
  }

  public Rotation2d getRotation() {
    return new Rotation2d(getTurningPosition());
  }

  // in meters per second.
  public double getDriveVelocity() {
    return m_driveVel.refresh().getValueAsDouble();
  }

  public void resetEncoders() {
    m_driveMotor.setPosition(0.0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getRotation());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(getDrivePosition(), getRotation());
  }


  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < DriveConstants.kTranslationalDeadbandMetersPerSecond) {
      stop();
      return;
    }

    state.optimize(getState().angle); //NEW check if working
    SmartDashboard.putNumber("Swerve/Speed/Commanded/Module_" + m_moduleId, state.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded/Angle_" + m_moduleId, state.angle.getRadians());
    SmartDashboard.putNumber("Swerve/Angle/Commanded/Module_" + m_moduleId, state.angle.getRadians());

    double ff = state.speedMetersPerSecond / DriveConstants.kMaxTranslationalMetersPerSecond;
    double pid = m_drivingPIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    m_driveMotor.set(ff + pid);
    m_turnMotor.set(m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians()));


    SmartDashboard.putString("Swerve_" + m_moduleId + "_state", state.toString());
  }


  public void stop() {
    m_driveMotor.set(0.0);
    m_turnMotor.set(0.0);
  }

  public StatusSignal<Angle> getDrivePos()
  {
    return m_drivePos;
  }

  public StatusSignal<AngularVelocity> getDriveVel()
  {
    return m_driveVel;
  }

  public double getDriveCurrent() {
    return m_driveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getTurnCurrent() {
    return m_turnMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turn" + m_moduleId, getTurningPosition());
    SmartDashboard.putNumber("Swerve/Speed/Measured/Module_" + m_moduleId, getDriveVelocity());
    SmartDashboard.putNumber("Swerve/Distance/Module_" + m_moduleId, getDrivePosition());
  }
} 
