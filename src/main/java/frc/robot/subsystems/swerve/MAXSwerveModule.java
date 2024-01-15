// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.ModuleConstants;

public class MAXSwerveModule {
  private final CANSparkMax m_drivingSparkMax;
  private final CANSparkMax m_turningSparkMax;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkPIDController m_drivingPIDController;
  private final SparkPIDController m_turningPIDController;
  private final SimpleMotorFeedforward m_SimpleMotorFeedforward;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  private final DoubleLogEntry m_driveMotorOutputLog;
  private final IntegerLogEntry m_driveMotorFaultsLog;
  private final DoubleLogEntry m_driveMotorSetpointLog;
  private final DoubleLogEntry m_driveMotorPositionLog;
  private final DoubleLogEntry m_driveMotorVelocityLog;
  private final DoubleLogEntry m_driveMotorCurrentLog;
  private final DoubleLogEntry m_driveMotorVoltageLog;
  private final DoubleLogEntry m_driveMotorTempLog;
  private final DoubleLogEntry m_driveCommandedVoltageLog;

  private final DoubleLogEntry m_turningMotorOutputLog;
  private final IntegerLogEntry m_turningMotorFaultsLog;
  private final DoubleLogEntry m_turningMotorSetpointLog;
  private final DoubleLogEntry m_turningMotorPositionLog;
  private final DoubleLogEntry m_turningMotorVelocityLog;
  private final DoubleLogEntry m_turningMotorCurrentLog;
  private final DoubleLogEntry m_turningMotorVoltageLog;
  private final DoubleLogEntry m_turningMotorTempLog;
  private final DoubleLogEntry m_turningCommandedVoltageLog;
  private final DoubleLogEntry m_turningMotorAbsoluteEncoderLog;



  private String m_label;
  private final DataLog m_log;
  private final DoubleLogEntry m_homeLog;
  private double m_home;


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, String label) {
    m_drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    m_drivingSparkMax.restoreFactoryDefaults();
    m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_drivingEncoder = m_drivingSparkMax.getEncoder();
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_drivingPIDController = m_drivingSparkMax.getPIDController();
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_drivingPIDController.setFeedbackDevice(m_drivingEncoder);
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    m_drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    m_drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    m_drivingPIDController.setP(ModuleConstants.kDrivingP);
    m_drivingPIDController.setI(ModuleConstants.kDrivingI);
    m_drivingPIDController.setD(ModuleConstants.kDrivingD);
    m_drivingPIDController.setFF(ModuleConstants.kDrivingFF);
    m_drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    //TODO: maybe making our turning feed forward negative? probably break something but its worth a shot
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingSparkMax.setIdleMode(ModuleConstants.kDrivingMotorIdleMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    m_drivingSparkMax.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_drivingSparkMax.burnFlash();
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);

    m_SimpleMotorFeedforward = new SimpleMotorFeedforward(0.13115, .40199); //ks: 0.16168 kv: 2.5'
    



    m_label = label;
    m_log = DataLogManager.getLog();
    m_home = Preferences.getDouble(m_label + ":home", 0.0);

    m_driveMotorOutputLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/output", m_label));
    m_driveMotorFaultsLog = new IntegerLogEntry(m_log, String.format("/swerve/%s/drive/faults", m_label));
    m_driveMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/setpoint", m_label));
    m_driveMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/position", m_label));
    m_driveMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/velocity", m_label));
    m_driveMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/current", m_label));
    m_driveMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/voltage", m_label));
    m_driveMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/temp", m_label));
    m_driveCommandedVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/drive/commandedVoltage", m_label));

    m_turningMotorOutputLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/output", m_label));
    m_turningMotorFaultsLog = new IntegerLogEntry(m_log, String.format("/swerve/%s/turn/faults", m_label));
    m_turningMotorSetpointLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/setpoint", m_label));
    m_turningMotorPositionLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/position", m_label));
    m_turningMotorVelocityLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/velocity", m_label));
    m_turningMotorCurrentLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/current", m_label));
    m_turningMotorVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/voltage", m_label));
    m_turningMotorTempLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/temp", m_label));
    m_turningCommandedVoltageLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/commandedVoltage", m_label));
    m_turningMotorAbsoluteEncoderLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/turn/absolute", m_label));
   
   
    m_homeLog = new DoubleLogEntry(m_log, String.format("/swerve/%s/home", m_label));

    logData();
    m_homeLog.append(m_home);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    m_drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity, 0, m_SimpleMotorFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond));
    m_turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double getAbsoluteEncoderZeroOffset()
  {
    return m_turningEncoder.getZeroOffset();
  }

  public double getEncoderCounts()
  {
    return m_drivingEncoder.getPositionConversionFactor() * m_drivingEncoder.getPosition();
  }

  public void setDriveVoltage(double voltage)
  {
    m_drivingSparkMax.setVoltage(voltage);
  }

  public void setTurnVoltage(double voltage)
  {
    m_turningSparkMax.setVoltage(voltage);
  }

  private void logData() {
    long timeStamp = (long) (Timer.getFPGATimestamp() * 1e6);
    
    m_driveMotorOutputLog.append(m_drivingSparkMax.getAppliedOutput(), timeStamp);
    m_driveMotorFaultsLog.append(m_drivingSparkMax.getFaults(), timeStamp);
    m_driveMotorPositionLog.append(m_drivingEncoder.getPosition(), timeStamp);
    m_driveMotorVelocityLog.append(m_drivingEncoder.getVelocity(), timeStamp);
    m_driveMotorCurrentLog.append(m_drivingSparkMax.getOutputCurrent(), timeStamp);
    m_driveMotorVoltageLog.append(m_drivingSparkMax.getBusVoltage(), timeStamp);
    m_driveMotorTempLog.append(m_drivingSparkMax.getMotorTemperature(), timeStamp);


    m_turningMotorOutputLog.append(m_turningSparkMax.getAppliedOutput(), timeStamp);
    m_turningMotorFaultsLog.append(m_turningSparkMax.getFaults(), timeStamp);
    m_turningMotorPositionLog.append(m_turningEncoder.getPosition(), timeStamp);
    m_turningMotorVelocityLog.append(m_turningEncoder.getVelocity(), timeStamp);
    m_turningMotorCurrentLog.append(m_turningSparkMax.getOutputCurrent(), timeStamp);
    m_turningMotorVoltageLog.append(m_turningSparkMax.getBusVoltage(), timeStamp);
    m_turningMotorTempLog.append(m_turningSparkMax.getMotorTemperature(), timeStamp);
   
}


public void runCharacterization(double volts, double offset){ 
  //reminder, this is different in AdvantageKit
 m_turningPIDController.setReference(offset, ControlType.kPosition);
  setDriveVoltage(volts);
}
public double getCharacterizationVelocity(){
  return m_drivingEncoder.getVelocity() * 2 * Math.PI; // rads per sec 
 }


}
