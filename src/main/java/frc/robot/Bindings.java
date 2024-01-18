// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
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

/** Add your docs here. */
public class Bindings {

    private static Boolean bCompressorEnabled = true;

    public Bindings() {}

        // TODO: test if this works! it should!
        public static void InitBindings(XboxController driverController, XboxController operatorController, 
            DriveSubsystem driveSubsystem, 
            TestMotors testMotors, 
            Pneumatics pneumatics, 
            Intake intake){
        
        new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new RunCommand(() -> { driveSubsystem.zeroHeading();}));
        
        // Both Trigger Control of Test Motors
        new Trigger(() -> {return (driverController.getRightTriggerAxis() > 0 || driverController.getLeftTriggerAxis() > 0);}).whileTrue(
        new RunCommand(() -> {testMotors.runMotors(() -> {return MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0, TestMotorConstants.kTestMotor1MaxSpeed);},
        () -> {return MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0, TestMotorConstants.kTestMotor2MaxSpeed);});},
        testMotors)).onFalse((new RunCommand(() -> {testMotors.stop();}, testMotors)));

        //Both Trigger Control of Test Flex Motors
        new Trigger(() -> {return (driverController.getRightTriggerAxis() > 0 || driverController.getLeftTriggerAxis() > 0);})
        .whileTrue(
        new RunCommand(() -> {testMotors.bothTriggerRunFlexMotors(() -> {return MathUtil.applyDeadband(driverController.getRightTriggerAxis(), 0, TestMotorConstants.kTestMotor1FlexMaxSpeed);},
        () -> {return MathUtil.applyDeadband(driverController.getLeftTriggerAxis(), 0, TestMotorConstants.kTestMotor2FlexMaxSpeed);});},
        testMotors)).onFalse((new RunCommand(() -> {testMotors.stop();}, testMotors)));

        // new Trigger(() -> {return (driverController.getRightTriggerAxis() > 0);}).whileTrue(new RunCommand(
        //   () -> {
        //     testMotors.runFlexMotors(driverController::getRightTriggerAxis);
        //   }, testMotors));
        
        



        //Compressor Toggle
        new Trigger(() -> {return driverController.getRightBumper();}).onTrue(new InstantCommand(() -> {
        if(bCompressorEnabled){
            pneumatics.disableCompressor();
            bCompressorEnabled = false;
        } else {
            pneumatics.enableCompressor();
            bCompressorEnabled = true;
        }
        }, pneumatics));

        // A toggle for DoubleSolenoid
        new Trigger(() -> {return driverController.getAButtonPressed();}).onTrue(new InstantCommand(() -> {
        pneumatics.getDoubleSolenoid().toggle();
        }, pneumatics));

        // B toggle for SingleSolenoid
        new Trigger(()-> {return driverController.getBButtonPressed();}).onTrue(new InstantCommand(() -> {
        pneumatics.getSolenoid().toggle();
        }, pneumatics));

        new Trigger(() -> {return driverController.getPOV() == 0;}).onTrue(new InstantCommand(() -> {
        intake.runIntakeAtSetSpeed();
        }, intake))
        .onFalse(new InstantCommand(() -> {intake.stopIntake();}, intake));

        new Trigger(() -> {return driverController.getLeftBumperPressed();}).toggleOnTrue(new InstantCommand(() -> {
        intake.runIntakeAtSetSpeed();
        })).toggleOnFalse(new InstantCommand(() -> {
        intake.stopIntake();
        }));


        //try if above doesnt work
        // new Trigger(() -> {return m_driverController.getLeftBumperPressed();}).onTrue(new InstantCommand(() -> {
        //   if(!bIntakeToggle){
        //     m_Intake.runIntakeAtSetSpeed();
        //     bIntakeToggle = true;
        //   } else {
        //     m_Intake.stopIntake();
        //     bIntakeToggle = false;
        //   }
        // }, m_Intake));

    

        new Trigger(() -> {return operatorController.getRightTriggerAxis() > .05;}).whileTrue(new RunCommand(() -> {
            intake.runIntake(() -> {return IntakeConstants.kIntakeSpeed;});
        }, intake));

        new Trigger(() -> {return operatorController.getLeftTriggerAxis() > .05;}).whileTrue(new RunCommand(() -> {
            intake.runIntake(() -> {return -IntakeConstants.kIntakeSpeed;});
        }, intake));

    
    }
}
