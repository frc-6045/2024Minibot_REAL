// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;




import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkFlex m_IntakeMotor;
  private final CANSparkFlex m_IndexerMotor;
  public Intake() {
    m_IntakeMotor = new CANSparkFlex(IntakeConstants.kIntakeCANID, MotorType.kBrushless);
    //m_Pneumatics.ActutateIntakeSolenoid();
    m_IndexerMotor = new CANSparkFlex(IntakeConstants.kIndexerCANID, MotorType.kBrushless);
  }

  public void runIntake(Supplier<Double> speed){
    if(speed.get() <= IntakeConstants.kIntakeSpeed){
      m_IntakeMotor.set(speed.get());
      m_IndexerMotor.set(speed.get());
    } else {
      m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
      m_IndexerMotor.set(IntakeConstants.kIntakeSpeed);
    }
  }

  public void runIntakeAtSetSpeed(){
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    m_IndexerMotor.set(IntakeConstants.kIntakeSpeed);

  }

  public void stopIntake() {
    m_IntakeMotor.set(0);
    m_IndexerMotor.set(0);
  }

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
