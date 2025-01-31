// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;


public class Pivot extends SubsystemBase {

  // sample pivot 

  private SparkMax m_PivotMotor = new SparkMax(PivotConstants.kPivotMotorID, MotorType.kBrushless);


  private DCMotorSim m_PivotSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, PivotConstants.kGearRatio), DCMotor.getNEO(1));
  



  // gonna add in an actual encoder
  private Encoder m_encoder = new Encoder(0, 1);
  private EncoderSim m_encoderSim = new EncoderSim(m_encoder);
  

  /** Creates a new Pivot. */
  public Pivot() {
    m_encoder.setDistancePerPulse(0.01); // encoder will track up to 0.01 of a "distance"
    m_encoderSim.setDistancePerPulse(0.01); // yeah

  }

  public void setPivotSpeed(double speed){
    m_PivotMotor.set(speed);

    m_PivotSim.setInputVoltage(speed*12); // since speed "maxes out" at 12V here, 0.3 would be 12*0.3 = 3.6V of input in this case (might be flawed but oh well)
  }
  

  public double getRotations(){
    // return m_encoder.getDistance();
    // return m_encoderSim.getDistance();
    return m_PivotSim.getAngularPositionRotations(); 
  }

  public DCMotorSim getPivotSim(){
    return m_PivotSim;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run



    m_PivotSim.setInputVoltage(m_PivotSim.getInputVoltage()); // idk why but bad code here we go
    m_PivotSim.update(0.020); // assume 20 ms loop time
    // maybe this'll work magically

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    // m_PivotSim.setAngle(m_PivotSim.getAngularPosition().times(kGearRatio));
    // m_PivotSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(kGearRatio));
  }
}
