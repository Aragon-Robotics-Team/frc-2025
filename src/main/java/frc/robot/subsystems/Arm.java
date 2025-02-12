// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {
  private TalonFX m_arm = new TalonFX(ArmConstants.ArmTalonDeviceId);
  private DigitalInput toplimitSwitch = new DigitalInput(ArmConstants.topLimitSwitchChannel);
  private DigitalInput bottomlimitSwitch = new DigitalInput(ArmConstants.bottomLimitSwitchChannel);
  private DutyCycleEncoder encoder = new DutyCycleEncoder(ArmConstants.encoderChannel);
  

  // we have removed the Arm constructor
  
  public void setSpeed(double m_speed){
    m_arm.set(m_speed);
  }

  public double getEncoderPosition(){
    return encoder.get();
  }

  public Boolean getTopLimitSwitch(){
    return toplimitSwitch.get();
  }

  public Boolean getBottomLimitSwitch(){
    return bottomlimitSwitch.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
