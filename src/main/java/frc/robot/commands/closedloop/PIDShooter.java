// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class PIDShooter extends Command {
  /** Creates a new PIDShooter. */
  private final Shooter m_Shooter;
  private final Feeder m_Feeder;
  private SparkPIDController m_PIDController;
  private SimpleMotorFeedforward m_Feedforward;
  private final CANSparkFlex shooterMotor;
  private final RelativeEncoder encoder;
  private double setpoint;
  private boolean atSetpoint;
  public PIDShooter(Shooter shooter, Feeder feeder, double setpoint) {
    m_Shooter = shooter;
    m_Feeder = feeder;
    shooterMotor = shooter.getMotor();
    encoder = shooterMotor.getEncoder(Type.kQuadrature, 8192); //TODO check type of encoder
    this.setpoint = setpoint;
    m_PIDController = shooterMotor.getPIDController();
    m_PIDController.setFeedbackDevice(encoder);
    m_Feedforward = new SimpleMotorFeedforward(0,0); //TODO characterization
    atSetpoint = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter, m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atSetpoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_PIDController.setReference(setpoint, ControlType.kVelocity, 0, m_Feedforward.calculate(encoder.getVelocity())); //TODO: characterization for feedforward
    System.out.println("velocity: " + encoder.getVelocity() + "| setpoint: " + setpoint);
    if(setpoint <= encoder.getVelocity()){
      m_Feeder.getMotor().set(ShooterConstants.kFeederSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public boolean getAtSetpoint() {
    return atSetpoint;
  }
}
