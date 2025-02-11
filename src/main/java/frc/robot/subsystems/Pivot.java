// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private SparkMax m_leftPivotMotor = new SparkMax(PivotConstants.kLeftPivotMotorID, MotorType.kBrushless);
  private SparkMax m_rightPivotMotor = new SparkMax(PivotConstants.kRightPivotMotorID, MotorType.kBrushless);
  private DCMotorSim m_pivotSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(2), 0.001, 10), DCMotor.getNEO(2));//ask design for gear ratio

  private Encoder m_encoder = new Encoder(0, 1);
  private EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  /** Creates a new Pivot. */
  public Pivot() {
    m_encoder.setDistancePerPulse(0.01);
    m_encoderSim.setDistancePerPulse(0.01);
  }

  public void setPivotSpeed(double speed) {
    m_leftPivotMotor.set(speed);
    m_rightPivotMotor.set(-speed);

    m_pivotSim.setInputVoltage(speed * 12);
  }

  public double getSimPivotPosition() {
    return m_pivotSim.getAngularPositionRotations();
  }

  public double getPivotPosition() {
    return m_encoder.get();
  }

  public DCMotorSim getPivotSim() {
    return m_pivotSim;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_pivotSim.setInputVoltage(m_pivotSim.getInputVoltage());
    m_pivotSim.update(0.02);
  }
}
