// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneButtonShooting extends SequentialCommandGroup {
  /** Creates a new OneButtonShooting. */
  private final DriveSubsystem m_drive;
  private final Shooter m_shooter;
  private final AngleController m_angleController;
  private final Feeder m_feeder;
  private final Intake m_intake;
  public OneButtonShooting(DriveSubsystem drive, AngleController angleController, Shooter shooter, Feeder feeder, Intake intake) {
    m_drive = drive;
    m_shooter = shooter;
    m_angleController = angleController;
    m_feeder = feeder;
    m_intake = intake;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TurnAndAim(m_angleController, m_drive), new PIDShooter(m_shooter, m_feeder, m_intake, -6000));
  }
}
