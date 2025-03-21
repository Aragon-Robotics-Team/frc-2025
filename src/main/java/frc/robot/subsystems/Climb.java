// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

// this is for our deep climb
public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  

  /*
   * requirements/specs:
    Climb uses a neo vortex with a 25:1 gearing with a spark flex base and a servo
    when button is pressed, arm should move out (to L1 position e.x.), motor should start spinning (to hook something)
    when button is pressed again, servo should release and motor should start spinning immediately to get the hook on the deep cage
  */

  private SparkFlex m_climbMotor = new SparkFlex(ClimbConstants.kClimbMotorID, MotorType.kBrushless);
  private SparkFlexConfig m_config = new SparkFlexConfig();

  private Servo m_servo = new Servo(ClimbConstants.kServoPWMPort);
  
  public Climb(){
    m_config.idleMode(IdleMode.kBrake);
    // idk if these limit constants are ok
    m_config.smartCurrentLimit(80, 60); // might be slightly unsafe
    m_climbMotor.configure(m_config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setClimbMotorSpeed(double speed){
    m_climbMotor.set(speed);
  }

  public double getMotorRotations(){
    return m_climbMotor.getEncoder().getPosition(); // i think this works -- TODO check what kind of encoder the spark flex has
  }

  public void setServoPosition(double pos){
    // pos should be between 0 and 1 where 0 corresponds with min position and 1 with max position
    m_servo.setPosition(pos);
  }

  public double getServoPosition(){
    return m_servo.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Servo Position", m_servo.get());
    SmartDashboard.putNumber("Climb Motor Rotations", getMotorRotations());
  }
}
