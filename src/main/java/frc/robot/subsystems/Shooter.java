// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private CANSparkFlex m_ShooterMotor;
  private CANSparkFlex m_FeederMotor;
  public Shooter() {
    m_ShooterMotor = new CANSparkFlex(ShooterConstants.kShooterMotorCANID, MotorType.kBrushless);
    m_FeederMotor = new CANSparkFlex(ShooterConstants.kFeederCANID, MotorType.kBrushless);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors(Supplier<Double> speedSupplier) {
    m_FeederMotor.set(speedSupplier.get());
    m_ShooterMotor.set(speedSupplier.get());
  }
  public CANSparkFlex[] getMotor(){
    CANSparkFlex[] array = {m_ShooterMotor, m_FeederMotor};
    return array;
    
  }
}
