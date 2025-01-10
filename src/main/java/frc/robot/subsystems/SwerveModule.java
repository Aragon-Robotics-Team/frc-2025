// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveModule extends SubsystemBase {
  private final TalonFX m_driveMotor;
  private final CANSparkMax m_turnMotor;

  private final DutyCycleEncoder m_absoluteEncoder;

  private final double m_absoluteEncoderOffset;
  private final PIDController m_drivingPIDController;
  private final PIDController m_turningPIDController;

  private final int m_moduleId;

  private StatusSignal<Double> m_drivePos;
  private StatusSignal<Double> m_driveVel;

  private Translation2d m_translation = new Translation2d();

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int turnId, int absoluteEncoderPort, double absoluteEncoderOffset,
      boolean driveReversed, boolean turningReversed, int moduleId) {
    // Initialize motors and encoders.
    m_driveMotor = new TalonFX(driveId);
    m_turnMotor = new CANSparkMax(turnId, MotorType.kBrushless);

    m_absoluteEncoder = new DutyCycleEncoder(new DigitalInput(absoluteEncoderPort));

    // Set conversion coefficients.
    
    m_absoluteEncoder.setDistancePerRotation(DriveConstants.kTurnEncoderPositionToRadians);

    // Initialize Everything else.
    m_absoluteEncoderOffset = absoluteEncoderOffset;
    m_turningPIDController = new PIDController(DriveConstants.kPTurning, DriveConstants.kITurning,
        DriveConstants.kDTurning);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    m_drivingPIDController = new PIDController(DriveConstants.kPDriving, DriveConstants.kIDriving, 
        DriveConstants.kDDriving);
    m_moduleId = moduleId;

    // Configure Turn Motor
    m_turnMotor.setIdleMode(IdleMode.kBrake);
    m_turnMotor.setInverted(turningReversed);
    //m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5,200);
    //m_turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 200);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.MotorOutput.Inverted = driveReversed ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.kDriveSensorToMechanismRatio;

    m_driveMotor.getConfigurator().apply(driveConfig);

    resetEncoders();

    m_drivePos = m_driveMotor.getPosition();
    m_driveVel = m_driveMotor.getVelocity();


    SmartDashboard.putData("Swerve/Distance/reset_" + m_moduleId,  new InstantCommand(() -> resetEncoders()));
    

  }

  // in meters.
  public double getDrivePosition() {
    return m_drivePos.refresh().getValueAsDouble();
  }

  public double getTurningPosition() {
    return m_absoluteEncoder.getDistance() - m_absoluteEncoderOffset;
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

  
  double rotation;
  double rotationCommand;
  double rotationPIDOutput;

  double velocity;
  double velocityCommand;

  public void setDesiredState(SwerveModuleState state) {
    if (Math.abs(state.speedMetersPerSecond) < DriveConstants.kTranslationalDeadbandMetersPerSecond) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    SmartDashboard.putNumber("Swerve/Speed/Commanded/Module_" + m_moduleId, state.speedMetersPerSecond);
    SmartDashboard.putNumber("Swerve/Commanded/Angle_" + m_moduleId, state.angle.getRadians());
    SmartDashboard.putNumber("Swerve/Angle/Commanded/Module_" + m_moduleId, state.angle.getRadians());

    double ff = state.speedMetersPerSecond / DriveConstants.kMaxTranslationalMetersPerSecond;
    double pid = m_drivingPIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    velocity = getDriveVelocity();
    velocityCommand = state.speedMetersPerSecond;

    

    m_driveMotor.set(ff + pid);
    m_turnMotor.set(m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians()));


    rotation = ((Math.toDegrees(((getRotation()).getRadians())) % 360 + 540)) % 360 - 180;
    rotationCommand = ((Math.toDegrees(((state.angle).getRadians())) % 360 + 540)) % 360 - 180;
    rotationPIDOutput = m_turningPIDController.calculate(getRotation().getRadians(), state.angle.getRadians());

    
    SmartDashboard.putString("Swerve_" + m_moduleId + "_state", state.toString());
  }

  public double getRotationActual()
  {
    return rotation;
  }

  public double getRotationCommand()
  {
    return rotationCommand;
  }

  public double getRotationPIDOutput()
  {
    return rotationPIDOutput;
  }

  public double getVelocityActual()
  {
    return velocity;
  }

  public double getVelocityCommand()
  {
    return velocityCommand;
  }

  




  public Translation2d getTranslation(){
    return new Translation2d(m_translation.getX(), m_translation.getY());
  }

  public void stop() {
    m_driveMotor.set(0.0);
    m_turnMotor.set(0.0);
  }

  public double getDriveCurrent() {
    return m_driveMotor.getStatorCurrent().getValueAsDouble();
  }

  public double getTurnCurrent() {
    return m_turnMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turn" + m_moduleId, getTurningPosition());
    SmartDashboard.putNumber("Swerve/Speed/Measured/Module_" + m_moduleId, getDriveVelocity());
    SmartDashboard.putNumber("Swerve/Distance/Module_" + m_moduleId, getDrivePosition());
  }
} 
