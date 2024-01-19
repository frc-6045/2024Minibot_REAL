// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
    private final CANSparkFlex m_FeederMotor;

  public Feeder() {
      m_FeederMotor = new CANSparkFlex(IntakeConstants.kFeederCANID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkFlex getMotor() {
    return m_FeederMotor;
  }
}
