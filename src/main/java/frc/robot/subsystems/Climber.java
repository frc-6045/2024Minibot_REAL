// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

//This will probably be real scuffed, sorry
public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final CANSparkFlex m_LeftClimbMotor;
  private final CANSparkFlex m_RightClimbMotor;
  
  public Climber() {
    m_LeftClimbMotor = new CANSparkFlex(ClimbConstants.kLeftClimbMotorCanId, MotorType.kBrushless);
    m_RightClimbMotor = new CANSparkFlex(ClimbConstants.kRightClimbMotorCanId, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runMotors(Supplier<Double> speedSupplier){
    m_LeftClimbMotor.set(speedSupplier.get());
    m_RightClimbMotor.set(-speedSupplier.get());
  }
}
