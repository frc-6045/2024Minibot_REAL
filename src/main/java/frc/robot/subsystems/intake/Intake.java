// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;




import java.util.function.Supplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Pneumatics;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkFlex m_IntakeMotor;
  private final CANSparkMax m_FeederMotor;
  private final Pneumatics m_Pneumatics;
  public Intake() {
    m_Pneumatics = new Pneumatics();
    m_IntakeMotor = new CANSparkFlex(IntakeConstants.kIntakeCANID, MotorType.kBrushless);
    //m_Pneumatics.ActutateIntakeSolenoid();
    m_FeederMotor = new CANSparkMax(IntakeConstants.kFeederCANID, MotorType.kBrushless);
  }

  public void runIntake(Supplier<Double> speed){
    m_IntakeMotor.set(speed.get());
    m_FeederMotor.set(speed.get());
  }

  public void runIntakeAtSetSpeed(){
    m_IntakeMotor.set(IntakeConstants.kIntakeSpeed);
    m_FeederMotor.set(IntakeConstants.kIntakeSpeed);
  }

  public void stopIntake() {
    m_IntakeMotor.set(0);
  }

   
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
