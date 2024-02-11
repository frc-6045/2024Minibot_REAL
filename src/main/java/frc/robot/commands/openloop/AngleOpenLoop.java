// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.openloop;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.closedloop.PIDAngleControl;
import frc.robot.subsystems.AngleController;

//Real basic stuff right now
public class AngleOpenLoop extends Command {
  private AngleController m_AngleController;
  private double AngleSpeed;
  /** Creates a new AngleOpenLoop. */
  public AngleOpenLoop(AngleController angleController, double AngleSpeed) {
    m_AngleController = angleController;
    this.AngleSpeed = AngleSpeed;
    addRequirements(m_AngleController);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AngleController.getAngleMotor().set(AngleSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AngleController.getAngleMotor().set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   // if(m_AngleController.getLimitSwitch().get()){
     // return true;
    //}
    return false;
  }
}
