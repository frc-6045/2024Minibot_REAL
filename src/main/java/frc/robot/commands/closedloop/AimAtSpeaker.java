// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.PoseMath;

public class AimAtSpeaker extends Command {
  /** Creates a new AimAtSpeaker. */
  private final DriveSubsystem m_drive;
  private Pose2d currentPose;
  private double setpointAngle;
  private PIDController m_AimPIDController;
  public AimAtSpeaker(DriveSubsystem drive) {
    m_drive = drive;
    setpointAngle = 0;
    currentPose = new Pose2d();
    m_AimPIDController = new PIDController(.03, 0, 0);
    m_AimPIDController.setTolerance(2);
    m_AimPIDController.enableContinuousInput(-180,180);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPose = m_drive.getPose();
    setpointAngle = PoseMath.getTargetAngle(FieldConstants.kSpeakerBackLocation, currentPose).getDegrees(); //FIXME: needs to work on red alliance 
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnSpeed = m_AimPIDController.calculate(m_drive.getPoseHeading(), setpointAngle);
    m_drive.drive(0, 0, turnSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_AimPIDController.atSetpoint();
  }
}
