// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax m_intakeMotor = new SparkMax(IntakeConstants.kIntakeMotorID, MotorType.kBrushless);
  
  // indexer is the thing in between the pivot/intake and the arm/elevator
  private SparkMax m_indexerMotor = new SparkMax(IntakeConstants.kIndexerMotorID, MotorType.kBrushless);


  public void setIntakeSpeed (double speed) {   
    m_intakeMotor.set(speed);
  }

  public void setIndexerSpeed (double speed) {
    m_indexerMotor.set(-speed);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
   
 