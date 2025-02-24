// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class EndEffector extends SubsystemBase {
  private SparkMax m_endEffectorMotor = new SparkMax(ArmConstants.EndEffectorMotorDeviceID, MotorType.kBrushless);

  /** Creates a new EndEffector. */
  public EndEffector() {}
  
  public void spinArmOuttakeMotor(double m_speed){
    m_endEffectorMotor.set(m_speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
