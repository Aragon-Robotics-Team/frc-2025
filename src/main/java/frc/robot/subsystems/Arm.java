// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;






import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ArmConstants;

public class Arm extends SubsystemBase {

  private TalonFX m_arm = new TalonFX(ArmConstants.kArmTalonDeviceId);
  private DigitalInput m_topLimitSwitch = new DigitalInput(ArmConstants.kTopLimitSwitchChannel);
  private DigitalInput m_bottomLimitSwitch = new DigitalInput(ArmConstants.kBottomLimitSwitchChannel);
  private DutyCycleEncoder m_encoder = new DutyCycleEncoder(ArmConstants.kEncoderChannel);
  private SparkMax m_armOuttakeMotor = new SparkMax(ArmConstants.ArmOuttakeMotorDeviceId, MotorType.kBrushless);

  // we have removed the Arm constructor
  public Arm(){
    m_arm.setNeutralMode(NeutralModeValue.Brake);
  }
  
  public void setSpeed(double speed){
    if ((getTopLimitSwitch() && speed > 0) || (getBottomLimitSwitch() && speed < 0)){
      speed = 0;
    }
    m_arm.set(speed);
    // System.out.println(speed);  
  }

  public double getEncoderPosition(){
    return m_encoder.get();
  }


  public Boolean getTopLimitSwitch(){
    return !m_topLimitSwitch.get();
  }

  public void spinArmOuttakeMotor(double m_speed){
    m_armOuttakeMotor.set(m_speed);
  }


  public Boolean getBottomLimitSwitch(){
    return !m_bottomLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm encoder ticks", getEncoderPosition());
    SmartDashboard.putBoolean("Top Arm Limit", getTopLimitSwitch());
    SmartDashboard.putBoolean("Bottom Arm Limit", getBottomLimitSwitch());
    // This method will be called once per scheduler run
  }
}
