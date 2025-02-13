// Copyright (c) FIRST and other WPILib contributors.                
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;





public class Elevator extends SubsystemBase {
  /** Creates a new Elevator.
   */
  private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchDio);
  private SparkMax m_elevator = new SparkMax(Constants.ElevatorConstants.deviceId, MotorType.kBrushless);
  private SparkMax m_elevator2Max = new SparkMax(Constants.ElevatorConstants.deviceId2, MotorType.kBrushless);

  private SparkMaxConfig brakeMode = new SparkMaxConfig();

  //public Elevator() {} elevator constructor is here 
  public Elevator(){
    // change this to be brake mode
    brakeMode.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_elevator.configure(brakeMode, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_elevator2Max.configure(brakeMode, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
  
  public void setSpeed(double speed) {
    m_elevator.set(speed);
    m_elevator2Max.set(-speed);
  }
  public double getElevatorPosition() {
    return m_elevator.getEncoder().getPosition();
  }
  public boolean getLimitSwitch(){
    return bottomLimitSwitch.get();
  }
  // public void SetNeutralMode(IdleMode neutralMode) {

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder ticks", getElevatorPosition());
  }
  public void addRequirements(){}
}