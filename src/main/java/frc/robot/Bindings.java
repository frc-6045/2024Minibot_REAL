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
import frc.robot.commands.openloop.ShooterOpenLoop;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.DriveSubsystem;

/** Add your docs here. */
public class Bindings {

    private static Boolean bCompressorEnabled = true;
    private static Boolean bIntakeToggle = false;

    public Bindings() {}

        // TODO: test if this works! it should!
        public static void InitBindings(XboxController driverController, XboxController operatorController, 
            DriveSubsystem driveSubsystem, 
            Shooter shooter,
            Pneumatics pneumatics, 
            Intake intake){
        
        new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new RunCommand(() -> { driveSubsystem.zeroHeading();}));
        
        new Trigger(() -> {return operatorController.getRightTriggerAxis() > 0;}).whileTrue(new ShooterOpenLoop(shooter, operatorController::getRightTriggerAxis));


        
        



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

        
        // // B toggle for SingleSolenoid
        // new Trigger(()-> {return driverController.getBButtonPressed();}).onTrue(new InstantCommand(() -> {
        // pneumatics.getSolenoid().toggle();
        // }, pneumatics));

        new Trigger(() -> {return driverController.getPOV() == 0;}).onTrue(new InstantCommand(() -> {
        intake.runIntakeAtSetSpeed();
        }, intake))
        .onFalse(new InstantCommand(() -> {intake.stopIntake();}, intake));

        // intake toggle
        //  new Trigger(() -> {return driverController.getLeftBumperPressed();}).onTrue(new InstantCommand(() -> {
        //    if(!bIntakeToggle){
        //      intake.runIntakeAtSetSpeed();
        //      bIntakeToggle = true;
        //    } else {
        //     intake.stopIntake();
        //      bIntakeToggle = false;
        //    }
        //  }, intake));

    

        new Trigger(() -> {return driverController.getRightTriggerAxis() > .05;}).whileTrue(new RunCommand(() -> {
            intake.runIntake(() -> {return IntakeConstants.kIntakeSpeed;});
        }, intake));

        new Trigger(() -> {return driverController.getLeftTriggerAxis() > .05;}).whileTrue(new RunCommand(() -> {
            intake.runIntake(() -> {return -IntakeConstants.kIntakeSpeed;});
        }, intake));

    
    }
}
