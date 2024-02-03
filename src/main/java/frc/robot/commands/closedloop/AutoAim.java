// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.PoseMath;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final DriveSubsystem m_Drive;
  private final Shooter m_Shooter;
  private PIDController m_ShooterAngleController;
  private PIDController m_TurningAngleController;
  private double shooterAngle;
  private double turningAngle;
  public AutoAim(DriveSubsystem drive, Shooter shooter) {
    m_Drive = drive;
    m_Shooter = shooter;

    m_ShooterAngleController = new PIDController(0, 0, 0);
    m_TurningAngleController = new PIDController(0, 0, 0);

    addRequirements(m_Drive, m_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterAngle = PoseMath.FindShootingAngle(m_Drive.getPose());
    turningAngle = PoseMath.FindTurningAngle(m_Drive.getPose());
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterAngleSpeed;
    double turningAngleSpeed;

    shooterAngleSpeed = m_ShooterAngleController.calculate(m_Shooter.getAngleEncoder().getPosition(), shooterAngle);
    turningAngleSpeed = m_TurningAngleController.calculate(m_Drive.getVisionAngle(), turningAngle);

    m_Shooter.getAngleMotor().set(shooterAngleSpeed);
    m_Drive.drive(0, 0, turningAngleSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_ShooterAngleController.atSetpoint() && m_TurningAngleController.atSetpoint());
  }
}
