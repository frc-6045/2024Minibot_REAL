// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class AngleController extends SubsystemBase {
  /** Creates a new AngleController. */
  private CANSparkFlex m_AngleMotor;

  private AbsoluteEncoder m_AngleEncoder;

  public AngleController() {
    m_AngleMotor = new CANSparkFlex(ShooterConstants.kAngleControlCANID, MotorType.kBrushless);
    m_AngleEncoder = m_AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public CANSparkFlex getAngleMotor() {
    return m_AngleMotor;
  }

  public AbsoluteEncoder getAngleEncoder() {
    return m_AngleEncoder;
  }
}
