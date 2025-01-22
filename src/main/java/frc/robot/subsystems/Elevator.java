// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private static final class Config{
    private static final int deviceId = 5;
  }
  /** Creates a new Elevator.
   */

  private SparkMax m_elevator = new SparkMax(Config.deviceId, MotorType.kBrushless);
  public Elevator() {}
  
  public void setSpeed(double speed) {
    m_elevator.set(speed);
  }
  public double getElevatorPosition() {
    return m_elevator.getEncoder().getPosition();
  }
  // public void SetNeutralMode(IdleMode neutralMode) {

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}