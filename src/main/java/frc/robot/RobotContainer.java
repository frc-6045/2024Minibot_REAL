// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.NamedCommands;

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
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.closedloop.PIDAngleControl;
import frc.robot.commands.closedloop.PIDShooter;
import frc.robot.subsystems.AngleController;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import frc.robot.subsystems.swerve.DriveSubsystem;
import frc.robot.util.LookupTables;
import frc.robot.util.PoseMath;

public class RobotContainer {
private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
private final XboxController m_driverController = new XboxController(0);
private final XboxController m_operatorController = new XboxController(1);
private final Shooter m_Shooter = new Shooter();
private final Feeder m_Feeder = new Feeder();
private final Pneumatics m_Pneumatics = new Pneumatics();
private final Intake m_Intake = new Intake();
private final Climber m_Climber = new Climber();
private final AngleController m_AngleController = new AngleController();
private final Trap m_Trap = new Trap();


private Autos m_autos = new Autos(m_driveSubsystem, m_Feeder, m_Intake, m_Pneumatics, m_Shooter);  
private ShuffleboardTab teleopTab = Shuffleboard.getTab("teleOp");
public RobotContainer() {

  NamedCommands.registerCommand("AngleAndShoot", new SequentialCommandGroup(new PIDAngleControl(m_AngleController, () -> {return LookupTables.getAngleValueAtDistance(PoseMath.getDistanceToSpeakerBack(m_driveSubsystem.getPose()));}), new PIDShooter(m_Shooter, m_Feeder, m_Intake, -6000)));
  NamedCommands.registerCommand("IntakeIn", new ParallelDeadlineGroup(new WaitCommand(1.5), new RunCommand(() -> {m_Intake.intakeIn();}, m_Intake)));
  NamedCommands.registerCommand("IntakeOut",new ParallelDeadlineGroup(new WaitCommand(1.5), new RunCommand(() -> {m_Intake.intakeOut();}, m_Intake)));;


    m_Pneumatics.setDefaultCommand(new RunCommand(() -> {
      if(m_driverController.getPOV() == 0) {
        m_Pneumatics.ActutateIntakeSolenoid(true);
      } else if(m_driverController.getPOV() == 180) {
        m_Pneumatics.ActutateIntakeSolenoid(false);
      }
    }, m_Pneumatics));

    m_driveSubsystem.setDefaultCommand(
    new RunCommand(
            () -> m_driveSubsystem.drive( //FIXME: very very stupid bodge
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.15),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.20),
                true),
            m_driveSubsystem));
    // m_AngleController.setDefaultCommand(
    //   new PIDAngleControl(m_AngleController, m_AngleController.getAngleEncoder().getPosition())
    // );
    
    


    configureBindings();
    teleopTab.addDouble("Right Trigger Axis", m_driverController::getRightTriggerAxis);
    teleopTab.addDouble("Left Trigger Axis", m_driverController::getLeftTriggerAxis);
    teleopTab.addDouble("Shooter RPM", () -> {return m_Shooter.getMotor().getEncoder().getVelocity();});
    teleopTab.addDouble("hood position", () -> {return m_AngleController.getAngleEncoder().getPosition();});
  }

  private void configureBindings() {
    Bindings.InitBindings(m_driverController, m_operatorController, 
    m_driveSubsystem, m_Shooter, 
    m_Feeder, m_Pneumatics, 
    m_AngleController, m_Intake, 
    m_Climber, m_Trap);
  }

  public Command getAutonomousCommand() {
    return m_autos.getAutonomousCommand();
  }
}