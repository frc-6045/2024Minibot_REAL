// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;


public class FeederOpenLoop extends Command {
  /** Creates a new FeederOpenLoop. */
  private Feeder m_Feeder;
  private Supplier<Double> speedSupplier;
  public FeederOpenLoop(Feeder feeder, Supplier<Double> speedSupplier) {
    m_Feeder = feeder;
    this.speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(speedSupplier.get() <= FeederConstants.kFeederSpeed){
      m_Feeder.getMotor().set(-speedSupplier.get());
    } else {
      m_Feeder.getMotor().set(-FeederConstants.kFeederSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Feeder.getMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
