// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants.PivotConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// simulation imports
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DutyCycleSim;
import edu.wpi.first.wpilibj.DigitalGlitchFilter;



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

    if ((speed < 0) && ((getTopLimitSwitch() || (getPivotPosition() < PivotConstants.kTopPivotRotations)))){
      m_pivotMotor.set(0);
      SmartDashboard.putNumber("Pivot Speed", 0);

      // if our pivot is at the bottom
    } else if ((speed > 0) && ((getBottomLimitSwitch() || (getPivotPosition() > PivotConstants.kBottomPivotRotations)))){
      m_pivotMotor.set(0);
      SmartDashboard.putNumber("Pivot Speed", 0);
    } else {
      m_pivotMotor.set(-speed);
      SmartDashboard.putNumber("Pivot Speed", speed);
    }
    
    // set a negative speed to sync up with the pivot speed graph
    SmartDashboard.putNumber("Actual Pivot Motor Speed", -m_pivotMotor.getAppliedOutput());
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

  //Speed in rotations per second
  public double getSpeed(){
    return m_pivotMotor.getEncoder().getVelocity()/60;
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
