// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class Arm extends SubsystemBase {
  private TalonFX m_arm = new TalonFX(Constants.ArmConstants.ArmTalonDeviceId);
  private DigitalInput toplimitSwitch = new DigitalInput(Constants.ArmConstants.topLimitSwitchChannel);
  private DigitalInput bottomlimitSwitch = new DigitalInput(Constants.ArmConstants.bottomLimitSwitchChannel);

  public Arm() {}

  public void setSpeed(double speed){
m_arm.set(speed);
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
