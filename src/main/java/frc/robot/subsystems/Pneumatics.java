// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PneumaticsConstants;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private final Compressor m_Compressor;
  private final Solenoid m_Solenoid;
  private final Solenoid m_TrapSolenoid;
  public Pneumatics() {
    m_Compressor = new Compressor(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH);
    m_Solenoid = new Solenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kSolenoidSingleChannel);
    m_TrapSolenoid = new Solenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kTrapSolenoidChannel);
    m_Compressor.enableDigital();
    System.out.println("enabled compressor");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void enableCompressor(){
    m_Compressor.enableDigital();
  }
  public void disableCompressor(){
    m_Compressor.disable();
  }

  public void ActutateIntakeSolenoid(boolean isOn) {
    m_Solenoid.set(isOn);
  }

  public void ActutateTrapSolenoid(boolean isOn){
    m_TrapSolenoid.set(isOn);
  }
  
  public void ToggleIntakeSolenoids(){
    m_Solenoid.toggle();
  }

  public void ToggleTrapSolenoid(){
    m_TrapSolenoid.toggle();
  }

  public Solenoid getSolenoid(){
    return m_Solenoid;
  }

  
}
