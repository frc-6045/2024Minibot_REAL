// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap;

public class TrapOpenLoop extends Command {
  /** Creates a new TrapOpenLoop. */
  private final Trap m_trap;
  private Supplier<Double> speedSupplier;
  public TrapOpenLoop(Trap trap, Supplier<Double> speedSupplier) {
    m_trap = trap;
    this.speedSupplier = speedSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_trap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_trap.runMotors(speedSupplier);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_trap.runMotors(() -> {return 0.0;});
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
