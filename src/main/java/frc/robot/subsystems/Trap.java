// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class Trap extends SubsystemBase {
  /** Creates a new Trap. */
  private final CANSparkFlex m_TrapMotor;
  public Trap() {
    m_TrapMotor = new CANSparkFlex(ClimbConstants.kTrapMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public CANSparkFlex getTrapMotor(){
    return m_TrapMotor;
  }

  public void runMotors(Supplier<Double> speedSupplier){
    m_TrapMotor.set(speedSupplier.get());
  }
}
