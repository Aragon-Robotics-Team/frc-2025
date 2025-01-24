// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

public class Pivot extends SubsystemBase {
  private SparkMax m_leftPivotMotor = new SparkMax(PivotConstants.kLeftPivotMotorID, MotorType.kBrushless);
  private SparkMax m_rightPivotMotor = new SparkMax(PivotConstants.kRightPivotMotorID, MotorType.kBrushless);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(0);
  /** Creates a new Pivot. */
  public Pivot() {}

  public void setPivotSpeed(double speed) {
    m_leftPivotMotor.set(speed);
    m_rightPivotMotor.set(-speed);
  }

  public double getPivotPosition(){
    return m_encoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
