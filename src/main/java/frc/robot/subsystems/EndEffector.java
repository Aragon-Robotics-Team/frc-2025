// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class EndEffector extends SubsystemBase {
  private SparkMax m_endEffectorMotor = new SparkMax(ArmConstants.kEndEffectorMotorDeviceID, MotorType.kBrushless);
  // note: the bream break, the green wire on the robot, is plugged into the FORWARD limit switch on the spark max
  private SparkLimitSwitch m_beamBreak = m_endEffectorMotor.getForwardLimitSwitch();

  /** Creates a new EndEffector. */
  public EndEffector() {}
  
  public void spinArmOuttakeMotor(double speed){
    // positive speed == spin inwards
    // if we hit the limit, then don't spin coral even more forwards

    
    if ((speed > 0) && (isBeamBreakTriggered())){
      System.out.println("Beambreak condition");
      m_endEffectorMotor.set(0);
    } else {
      m_endEffectorMotor.set(speed);
    }
    
    // m_endEffectorMotor.set(speed);
  }

  public boolean isBeamBreakTriggered(){
    return m_beamBreak.isPressed();
  }


  @Override
  public void periodic() {    
    SmartDashboard.putBoolean("Beakbreak", isBeamBreakTriggered());
    // This method will be called once per scheduler run
  }
}
