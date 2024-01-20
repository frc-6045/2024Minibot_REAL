// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class ShooterOpenLoop extends Command {
  /** Creates a new ShooterOpenLoop. */
  private Shooter m_Shooter;
  private Supplier<Double> speedSupplier;
  public ShooterOpenLoop(Shooter shooter, Supplier<Double> speedSupplier) {
    m_Shooter = shooter;
    this.speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     if(speedSupplier.get() <=  ShooterConstants.kShooterMaxSpeed){
      m_Shooter.getMotor()[0].set(speedSupplier.get());
      m_Shooter.getMotor()[1].set(speedSupplier.get());

    } else {
      m_Shooter.getMotor()[0].set(ShooterConstants.kShooterMaxSpeed);
      m_Shooter.getMotor()[1].set(ShooterConstants.kShooterMaxSpeed);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_Shooter.getMotor()[0].set(0);
      m_Shooter.getMotor()[1].set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
