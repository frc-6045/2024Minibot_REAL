// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.closedloop.PIDShooter;
import frc.robot.commands.openloop.FeederOpenLoop;
import frc.robot.commands.openloop.IntakeOpenLoop;
import frc.robot.commands.openloop.ShooterAndFeederOpenLoop;
import frc.robot.commands.openloop.ShooterOpenLoop;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.DriveSubsystem;

/** Add your docs here. */
// Henry's Comment
public class Bindings {

    private static boolean bCompressorEnabled = true;
    private static boolean bIntakeToggle = false;
    private static boolean bFeederToggle = false;


    public Bindings() {}

        
        public static void InitBindings(XboxController driverController, XboxController operatorController, 
            DriveSubsystem driveSubsystem, 
            Shooter shooter,
            Feeder feeder,
            Pneumatics pneumatics, 
            Intake intake){
        
        new JoystickButton(driverController, XboxController.Button.kStart.value).onTrue(new InstantCommand(() -> { driveSubsystem.zeroHeading();}, driveSubsystem));
        


        //shooter
        new Trigger(() -> {return operatorController.getAButton();}).whileTrue(new FeederOpenLoop(feeder, () -> {return FeederConstants.kFeederSpeed;}));
        
        new Trigger(() -> {return operatorController.getLeftTriggerAxis() > 0;}).whileTrue(new ShooterOpenLoop(shooter, operatorController::getLeftTriggerAxis));

        new Trigger(() -> {return operatorController.getRightTriggerAxis() > 0;}).whileTrue(new ShooterAndFeederOpenLoop(shooter, feeder, operatorController::getRightTriggerAxis, operatorController::getRightTriggerAxis));
        
        new Trigger(() -> {return operatorController.getXButton();}).whileTrue(new PIDShooter(shooter, feeder,intake, 5400)); //TODO: EXTREMELY WIP, DO NOT USE YET. setpoint in rpm

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
        
        //intake solenoids (up is down, down is up! i hate it)
        new Trigger(() -> {return driverController.getPOV() == 0;}).onTrue(new InstantCommand(() -> {
            pneumatics.ActutateIntakeSolenoid(true);
        }, intake));

           new Trigger(() -> {return driverController.getPOV() == 360;}).onTrue(new InstantCommand(() -> {
            pneumatics.ActutateIntakeSolenoid(false);
        }, intake));

        // // intake toggle
         new Trigger(() -> {return driverController.getLeftBumperPressed();}).onTrue(new InstantCommand(() -> {
           if(!bIntakeToggle){
             intake.runIntakeAtSetSpeed();
             bIntakeToggle = true;
           } else {
            intake.stopIntake();
             bIntakeToggle = false;
           }
         }, intake));

    

         new Trigger(() -> {return driverController.getRightTriggerAxis() > .05;}).whileTrue(new IntakeOpenLoop(intake, driverController::getRightTriggerAxis));

         new Trigger(() -> {return driverController.getLeftTriggerAxis() > .05;}).whileTrue(new IntakeOpenLoop(intake, () -> {return -driverController.getLeftTriggerAxis();}));

        
        //  new Trigger(() -> {return driverController.getRightTriggerAxis() > 0 || driverController.getLeftTriggerAxis() > 0;}).whileTrue(new RunCommand(() -> {
        //     intake.testRunMotors(driverController::getRightTriggerAxis, driverController::getLeftTriggerAxis);
        //  }, intake)).onFalse(new InstantCommand(() -> {
        //     intake.stopIntake();
        //  }));

        //  new Trigger(() -> {return driverController.getPOV() == 90 || driverController.getPOV() == 270;}).whileTrue(new RunCommand(() -> {
        //     if(driverController.getPOV() == 90) {
        //         intake.testRunMotors(() -> {return -IntakeConstants.kIntakeSpeed;}, () -> {return 0.0;});
        //     } else {
        //         intake.testRunMotors(() -> {return 0.0;}, () -> {return -IntakeConstants.kIndexerSpeed;});
        //     }
        //  }, intake)).onFalse(new InstantCommand(() -> {
        //     intake.stopIntake();
        //  }));
      } 

    
}
