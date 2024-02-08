// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.closedloop;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.swerve.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurnAndAim extends ParallelCommandGroup {
  /** Creates a new TurnAndAim. */
  private final AngleController m_angleController;
  private final DriveSubsystem m_drive;
  public TurnAndAim(AngleController angleController, DriveSubsystem drive) {
    m_angleController = angleController;
    m_drive = drive;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new PIDAngleControl(m_angleController, 0), new AimAtSpeaker(m_drive));
  }
}
