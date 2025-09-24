// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class YAGSLSwerveModule extends SubsystemBase {

  private TalonFX m_driveMotor;
  private TalonFX m_steerMotor;
  private DutyCycleEncoder m_turnEncoder;

  /** Creates a new YAGSLSwerveModule. */
  public YAGSLSwerveModule(int driveMoterID, int steerMotorID, int turnEncoderID) {
    m_driveMotor = new TalonFX(driveMoterID);
    m_steerMotor = new TalonFX(steerMotorID);
    m_turnEncoder = new DutyCycleEncoder(turnEncoderID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
