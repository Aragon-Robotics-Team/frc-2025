// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {

  private SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorID, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();

  /** Creates a new Indexer. */
  public Indexer() {
    config.smartCurrentLimit(25); // 25a
    // really draws 12a max cause we limit the speed to 0.6
    m_indexerMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
  
  public void setIndexerSpeed (double speed) {
    m_indexerMotor.set(-speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
