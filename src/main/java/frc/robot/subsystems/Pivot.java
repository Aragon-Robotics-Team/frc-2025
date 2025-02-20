// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private SparkMax m_pivotMotor = new SparkMax(PivotConstants.kPivotMotorID, MotorType.kBrushless);
  //private DCMotorSim m_pivotSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(PivotConstants.kNumMotors), 0.001, PivotConstants.kGearRatio), DCMotor.getNEO(PivotConstants.kNumMotors));//ask design for gear ratio

  // with this offset, when our pivot is fully down, that is at "1" rotation (0.999)
  // when our pivot is fully up, that is at ~0.672 rotations
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(PivotConstants.kPivotEncoderChannel, 1, 0.1385);

  private DigitalInput topLimitSwitch = new DigitalInput(PivotConstants.kTopLimitSwitchID); // 6
  private DigitalInput bottomLimitSwitch = new DigitalInput(PivotConstants.kBottomLimitSwitchID); // 8


  private SparkMaxConfig brakeMode = new SparkMaxConfig();

  /** Creates a new Pivot. */
  public Pivot() {
    brakeMode.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_pivotMotor.configure(brakeMode, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }

  public void setPivotSpeed(double speed) {
    // negative speed -- move pivot up
    // positive speed -- move pivot down (to intake)
    m_pivotMotor.set(-speed);

    //m_pivotSim.setInputVoltage(speed * 12);
  }


  public boolean getTopLimitSwitch(){
    return !topLimitSwitch.get();
  }

  public boolean getBottomLimitSwitch(){
    return !bottomLimitSwitch.get();
  }

  /*public double getSimPivotPosition() {
    return m_pivotSim.getAngularPositionRotations();
  }*/

  public double getPivotPosition() {
    return m_encoder.get();
  }

  /*public DCMotorSim getPivotSim() {
    return m_pivotSim;
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //m_pivotSim.setInputVoltage(m_pivotSim.getInputVoltage());
    //m_pivotSim.update(0.02);
  }
}
