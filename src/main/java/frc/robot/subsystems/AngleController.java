// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class AngleController extends SubsystemBase {
  /** Creates a new AngleController. */
  private CANSparkFlex m_AngleMotor;
  private DigitalInput m_limitSwitch;
  private AbsoluteEncoder m_AngleEncoder;
  private RelativeEncoder m_RelativeEncoder;

  public AngleController() {
    m_AngleMotor = new CANSparkFlex(ShooterConstants.kAngleControlCANID, MotorType.kBrushless);
    m_AngleEncoder = m_AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
    //m_RelativeEncoder = m_AngleMotor.getEncoder();
    m_limitSwitch = new DigitalInput(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("hood angle", m_AngleEncoder.getPosition());
  }

  public CANSparkFlex getAngleMotor() {
    return m_AngleMotor;
  }

  //public RelativeEncoder getAngleEncoder() {
    //return m_AngleEncoder;
  //  return m_RelativeEncoder;
  //}

  public AbsoluteEncoder getAngleEncoder() {
    return m_AngleEncoder;
  }

  public DigitalInput getLimitSwitch(){
    return m_limitSwitch;
  }
}
