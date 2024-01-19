// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
private final XboxController m_driverController = new XboxController(0);
private final XboxController m_operatorController = new XboxController(1);
private final Shooter m_Shooter = new Shooter();
private final Pneumatics m_Pneumatics = new Pneumatics();
private final Intake m_Intake = new Intake();
//state checks
private Boolean bCompressorEnabled = true;
private Boolean bIntakeToggle = false;

private Autos m_autos = new Autos(m_driveSubsystem);  
private ShuffleboardTab teleopTab = Shuffleboard.getTab("teleOp");
public RobotContainer() {
    m_Pneumatics.setDefaultCommand(new RunCommand(() -> {
      if(m_driverController.getPOV() == 0) {
        m_Pneumatics.ActutateIntakeSolenoid(true);
      } else if(m_driverController.getPOV() == 180) {
        m_Pneumatics.ActutateIntakeSolenoid(false);
      }
    }, m_Pneumatics));

    m_driveSubsystem.setDefaultCommand(
    new RunCommand(
            () -> m_driveSubsystem.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.20),
                true),
            m_driveSubsystem));


    configureBindings();
    teleopTab.addDouble("Right Trigger Axis", m_driverController::getRightTriggerAxis);
    teleopTab.addDouble("Left Trigger Axis", m_driverController::getLeftTriggerAxis);
    
  }

  private void configureBindings() {
    Bindings.InitBindings(m_driverController, m_operatorController, m_driveSubsystem, m_Shooter, m_Pneumatics, m_Intake);
  }

  public Command getAutonomousCommand() {
    return m_autos.getAutonomousCommand();
  }
}