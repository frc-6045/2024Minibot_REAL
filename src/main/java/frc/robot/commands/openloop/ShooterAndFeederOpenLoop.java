// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class ShooterAndFeederOpenLoop extends Command {
  private final Shooter m_Shooter;
  private final Feeder m_Feeder;
  private Supplier<Double> shooterSpeed;
  private Supplier<Double> feederSpeed;
  /** Creates a new ShooterAndFeederOpenLoop. */
  public ShooterAndFeederOpenLoop(Shooter shooter, Feeder feeder, Supplier<Double> shooterSpeed, Supplier<Double> feederSpeed) {
    m_Shooter = shooter;
    m_Feeder = feeder;
    this.shooterSpeed = shooterSpeed;
    this.feederSpeed = feederSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter, m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooterSpeed.get() <=  ShooterConstants.kShooterMaxSpeed){
      m_Shooter.getMotor().set(shooterSpeed.get());
    } else {
      m_Shooter.getMotor().set(ShooterConstants.kShooterMaxSpeed);
    }

    if(feederSpeed.get() <= FeederConstants.kFeederSpeed){
      m_Feeder.getMotor().set(-feederSpeed.get());
    } else {
      m_Feeder.getMotor().set(-FeederConstants.kFeederSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shooter.getMotor().set(0);
    m_Feeder.getMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
