// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.encoders.CANCoderSwerve;
import swervelib.encoders.DIODutyCycleEncoderSwerve;
import swervelib.encoders.TalonSRXEncoderSwerve;

public class YAGSLSwerveModule extends SubsystemBase {

  private TalonFX m_driveMotor;
  private TalonFX m_steerMotor;
  private DIODutyCycleEncoderSwerve m_absoluteEncoder;

  /** Creates a new YAGSLSwerveModule. */
  public YAGSLSwerveModule(int driveMoterID, int steerMotorID, int absoluteEncoderID) {
    m_driveMotor = new TalonFX(driveMoterID);
    m_steerMotor = new TalonFX(steerMotorID);

    m_absoluteEncoder = new DIODutyCycleEncoderSwerve(absoluteEncoderID);
    m_absoluteEncoder.factoryDefault();

    m_absoluteEncoder.getAbsoluteEncoder();
    m_absoluteEncoder.configure(false);
    

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  }

  public double getDistance() {
    return 0; // edit later
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
