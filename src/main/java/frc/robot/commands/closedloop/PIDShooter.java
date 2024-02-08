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
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class PIDShooter extends Command {
  /** Creates a new PIDShooter. */
  private final Shooter m_Shooter;
  private final Feeder m_Feeder;
  private final Intake m_Intake;
   private SparkPIDController m_PIDController;
 
  private SimpleMotorFeedforward m_Feedforward;
  private final CANSparkFlex shooterMotor;
  private final RelativeEncoder encoder;
  private double setpoint;
  private boolean atSetpoint;
  public PIDShooter(Shooter shooter, Feeder feeder, Intake intake, double setpoint) {
    m_Shooter = shooter;
    m_Feeder = feeder;
    m_Intake = intake;
    shooterMotor = shooter.getMotor();
    encoder = shooterMotor.getEncoder(); //TODO check type of encoder
    this.setpoint = setpoint;
  
    
    m_PIDController = shooterMotor.getPIDController();
    m_PIDController.setP(ShooterConstants.kShooterP);
    m_PIDController.setI(ShooterConstants.kShooterI);
    m_PIDController.setD(ShooterConstants.kShooterD);
    m_PIDController.setIZone(0);
    m_PIDController.setFF(.0001);
    m_PIDController.setOutputRange(-1, 1);

    m_PIDController.setFeedbackDevice(encoder);
    m_Feedforward = new SimpleMotorFeedforward(.17674,.00030); //TODO characterization
    atSetpoint = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter, m_Feeder, m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    atSetpoint = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   
 
    m_PIDController.setReference(setpoint, ControlType.kVelocity, 0, m_Feedforward.calculate(setpoint)); //TODO: characterization for feedforward

   System.out.println(encoder.getVelocity());
    if(encoder.getVelocity() <= -ShooterConstants.kShooterLaunchRPM){
      m_Feeder.getMotor().set(FeederConstants.kFeederSpeed);               
      m_Intake.runIntake(() -> {return -IntakeConstants.kIntakeSpeed;});
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterMotor.set(0);
    m_Feeder.getMotor().set(0);
    m_Intake.stopIntake();
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
