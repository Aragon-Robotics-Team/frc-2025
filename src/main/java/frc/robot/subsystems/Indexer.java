// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {

  private SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorID, MotorType.kBrushless);

  /** Creates a new Indexer. */
  public Indexer() {}
  
  public void setIndexerSpeed (double speed) {
    m_indexerMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
