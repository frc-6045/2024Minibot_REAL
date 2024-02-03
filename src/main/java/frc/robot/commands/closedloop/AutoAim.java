// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.PoseMath;
import frc.robot.util.Vision;

public class AutoAim extends Command {
  /** Creates a new AutoAim. */
  private final DriveSubsystem m_Drive;
  private final Shooter m_Shooter;
  private final Vision m_Vision;
  private PIDController m_ShooterAngleController;
  private PIDController m_TurningAngleController;
  private double shooterAngle;
  private double turningAngle;
  public AutoAim(DriveSubsystem drive, Shooter shooter, Vision vision) {
    m_Drive = drive;
    m_Shooter = shooter;
    m_Vision = vision;
    m_ShooterAngleController = new PIDController(DriveConstants.kTurningAngleP, DriveConstants.kTurningAngleI,DriveConstants.kTurningAngleD);
    m_TurningAngleController = new PIDController(ShooterConstants.kShooterAngleP, ShooterConstants.kShooterAngleI, ShooterConstants.kShooterAngleD);

    m_ShooterAngleController.setTolerance(.1);
    m_TurningAngleController.setTolerance(5);
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
    turningAngleSpeed = m_TurningAngleController.calculate(m_Vision.getVisionPose().getRotation().getDegrees(), turningAngle);

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
