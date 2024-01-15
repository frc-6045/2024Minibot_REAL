// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TriggerWheels extends SubsystemBase {
  /** Creates a new TriggerWheels. */
  CANSparkBase m_triggerMotor1;
  CANSparkBase m_triggerMotor2;
  public TriggerWheels() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
