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
import frc.robot.Constants.TestMotorConstants;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.TestMotors;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DriveSubsystem;

public class RobotContainer {
private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
private final XboxController m_driverController = new XboxController(0);
private final XboxController m_operatorController = new XboxController(1);
private final TestMotors m_TestMotors = new TestMotors();
private final Pneumatics m_Pneumatics = new Pneumatics();
private final Intake m_Intake = new Intake();
//state checks
private Boolean bCompressorEnabled = true;
private Boolean bIntakeToggle = false;

private Autos m_autos = new Autos(m_driveSubsystem, m_TestMotors);  
private ShuffleboardTab teleopTab = Shuffleboard.getTab("teleOp");
public RobotContainer() {

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
    Bindings.InitBindings(m_driverController, m_operatorController, m_driveSubsystem, m_TestMotors, m_Pneumatics, m_Intake);
    // new JoystickButton(m_driverController, XboxController.Button.kStart.value).onTrue(new RunCommand(() -> { m_driveSubsystem.zeroHeading();}));
    
    // // Both Trigger Control of Test Motors
    // new Trigger(() -> {return (m_driverController.getRightTriggerAxis() > 0 || m_driverController.getLeftTriggerAxis() > 0);}).whileTrue(
    // new RunCommand(() -> {m_TestMotors.runMotors(() -> {return MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), 0, TestMotorConstants.kTestMotor1MaxSpeed);},
    //  () -> {return MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), 0, TestMotorConstants.kTestMotor2MaxSpeed);});},
    //   m_TestMotors)).onFalse((new RunCommand(() -> {m_TestMotors.stop();}, m_TestMotors)));

    // //Both Trigger Control of Test Flex Motors
    // new Trigger(() -> {return (m_driverController.getRightTriggerAxis() > 0 || m_driverController.getLeftTriggerAxis() > 0);})
    // .whileTrue(
    // new RunCommand(() -> {m_TestMotors.bothTriggerRunFlexMotors(() -> {return MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), 0, TestMotorConstants.kTestMotor1FlexMaxSpeed);},
    //  () -> {return MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), 0, TestMotorConstants.kTestMotor2FlexMaxSpeed);});},
    //   m_TestMotors)).onFalse((new RunCommand(() -> {m_TestMotors.stop();}, m_TestMotors)));

    // // new Trigger(() -> {return (m_driverController.getRightTriggerAxis() > 0);}).whileTrue(new RunCommand(
    // //   () -> {
    // //     m_TestMotors.runFlexMotors(m_driverController::getRightTriggerAxis);
    // //   }, m_TestMotors));
    
    
    // // new Trigger(() -> {return (m_driverController.getLeftTriggerAxis() > 0);}).whileTrue(new RunCommand(() -> {
    // //   m_TestMotors.OneSupplierRunMotors(m_driverController::getLeftTriggerAxis);
    // // }, m_TestMotors));


    // //Compressor Toggle
    // new Trigger(() -> {return m_driverController.getRightBumper();}).onTrue(new InstantCommand(() -> {
    //   if(bCompressorEnabled){
    //     m_Pneumatics.disableCompressor();
    //     bCompressorEnabled = false;
    //   } else {
    //     m_Pneumatics.enableCompressor();
    //     bCompressorEnabled = true;
    //   }
    // }, m_Pneumatics));

    // // A toggle for DoubleSolenoid
    // new Trigger(() -> {return m_driverController.getAButtonPressed();}).onTrue(new InstantCommand(() -> {
    //  m_Pneumatics.getDoubleSolenoid().toggle();
    // }, m_Pneumatics));

    // // B toggle for SingleSolenoid
    // new Trigger(()-> {return m_driverController.getBButtonPressed();}).onTrue(new InstantCommand(() -> {
    //   m_Pneumatics.getSolenoid().toggle();
    // }, m_Pneumatics));

    // new Trigger(() -> {return m_driverController.getPOV() == 0;}).onTrue(new InstantCommand(() -> {
    //   m_Intake.runIntakeAtSetSpeed();
    // }, m_Intake))
    // .onFalse(new InstantCommand(() -> {m_Intake.stopIntake();}, m_Intake));

    // new Trigger(() -> {return m_driverController.getLeftBumperPressed();}).toggleOnTrue(new InstantCommand(() -> {
    //   m_Intake.runIntakeAtSetSpeed();
    // })).toggleOnFalse(new InstantCommand(() -> {
    //   m_Intake.stopIntake();
    // }));


    // //try if above doesnt work
    // // new Trigger(() -> {return m_driverController.getLeftBumperPressed();}).onTrue(new InstantCommand(() -> {
    // //   if(!bIntakeToggle){
    // //     m_Intake.runIntakeAtSetSpeed();
    // //     bIntakeToggle = true;
    // //   } else {
    // //     m_Intake.stopIntake();
    // //     bIntakeToggle = false;
    // //   }
    // // }, m_Intake));

    

    // new Trigger(() -> {return m_operatorController.getRightTriggerAxis() > .95;}).whileTrue(new RunCommand(() -> {
    //   m_Intake.runIntake(() -> {return IntakeConstants.kIntakeSpeed;});
    // }, m_Intake));

    // new Trigger(() -> {return m_operatorController.getLeftTriggerAxis() > .95;}).whileTrue(new RunCommand(() -> {
    //     m_Intake.runIntake(() -> {return -IntakeConstants.kIntakeSpeed;});
    // }, m_Intake));
  }



  public Command getAutonomousCommand() {
    return m_autos.getAutonomousCommand();
  }
}