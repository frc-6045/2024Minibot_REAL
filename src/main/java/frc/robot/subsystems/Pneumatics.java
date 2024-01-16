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
  private final DoubleSolenoid m_DoubleSolenoid; //change to doublesolenoid
  public Pneumatics() {
    m_Compressor = new Compressor(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH);
    m_DoubleSolenoid = new DoubleSolenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kDoubleSolenoidForwardChannel, PneumaticsConstants.kDoubleSolenoidReverseChannel);
    m_Solenoid = new Solenoid(PneumaticsConstants.kPneumaticsModuleCANID, PneumaticsModuleType.REVPH, PneumaticsConstants.kSolenoidSingleChannel);
    m_Compressor.enableDigital();
    System.out.println("enabled compressor");
    m_DoubleSolenoid.set(Value.kReverse);
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

  public void setSolenoid(Supplier<Boolean> bool0, Supplier<Boolean> bool1){
    if(bool0.get()){
      System.out.println("forward");
      m_DoubleSolenoid.set(Value.kForward);
    } else if(bool1.get()){
      System.out.println("reverse");
      m_DoubleSolenoid.set(Value.kReverse);
    } else {
      System.out.println("off");
      m_DoubleSolenoid.set(Value.kOff);
    }
  }
  public Solenoid getSolenoid(){
    return m_Solenoid;
  }
  public DoubleSolenoid getDoubleSolenoid(){
    return m_DoubleSolenoid;
  }
}
