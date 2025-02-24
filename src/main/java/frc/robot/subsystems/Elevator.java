// Copyright (c) FIRST and other WPILib contributors.                
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;








public class Elevator extends SubsystemBase {
  /** Creates a new Elevator.
   */

  //I changed the names of the motors because they are confusing.
  //In general, m_elevator should refer to the Elevator subsystem being used in other files (Commands, RobotContainer, etc.)

  private DigitalInput bottomLimitSwitch = new DigitalInput(Constants.ElevatorConstants.limitSwitchDio);
  private SparkMax m_neo1 = new SparkMax(Constants.ElevatorConstants.deviceId, MotorType.kBrushless);
  private SparkMax m_neo2 = new SparkMax(Constants.ElevatorConstants.deviceId2, MotorType.kBrushless);

  private SparkMaxConfig brakeMode = new SparkMaxConfig();

  //public Elevator() {} elevator constructor is here 
  public Elevator(){
    // change this to be brake mode
    brakeMode.idleMode(SparkBaseConfig.IdleMode.kBrake);
    m_neo1.configure(brakeMode, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
    m_neo2.configure(brakeMode, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
  }
  
  public void setSpeed(double speed) {
    m_neo1.set(speed);
    m_neo2.set(-speed);
    SmartDashboard.putNumber("setSpeed", speed);
  }


  public double getSpeed(){
    //getVelocity returns RPM, so /60 converts it to RPS.
    return m_neo1.getEncoder().getVelocity()/60;

  }

  public double getElevatorPosition() {
    return m_neo1.getEncoder().getPosition();
  }
  
  public boolean getLimitSwitch(){
    return bottomLimitSwitch.get();
  }

  public void setElevatorPosition(double position) {
    m_neo1.getEncoder().setPosition(position);
  }
  // public void SetNeutralMode(IdleMode neutralMode) {

  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder ticks", getElevatorPosition());
    SmartDashboard.putNumber("elevator speed", getSpeed());
  }
  public void addRequirements(){}
}