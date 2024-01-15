// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    
    //Slew Constants
    public static final double kMaxSpeedMetersPerSecond = 4.8; //changed from 4.8 bc stuff wasnt straped down TODO: do that
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
    kMaxAngularSpeed / 4; // was 4
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;


    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5); //22.5
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.7); // 26.7
    // Distance between front and back wheels on robot
    public static final double radiusMeters = 0.8879;
    //the distance from the center of the robot to the furthest swerve module
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));


    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; //-Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 6; //6
    public static final int kRearRightTurningCanId = 8; //8

    public static final boolean kGyroReversed = false;
    //change
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

        //TODO: maybe remove this multiplier
    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI ) 
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.07; //was 0.04
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 0; 
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;


    // public static final double kDrivingP = 0.07; //was 0.04
    // public static final double kDrivingI = 0;
    // public static final double kDrivingD = 0;
    // public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps; 
    // public static final double kDrivingMinOutput = -1;
    // public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 01; //was 1
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = -0.00; //FIXME: changed feed forward from 0
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class AutoConstants {


    public static final HolonomicPathFollowerConfig autoBuilderPathConfig = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
      new PIDConstants(5.0, 0.0 ,0.2), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
      new PIDConstants(1.5, 0.0, 0), //5.0, 0, 0.2
      DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
      DriveConstants.radiusMeters, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig());

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    public static final TrapezoidProfile.Constraints kDriveControllerConstraints = new TrapezoidProfile.Constraints(
    DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);

    public static final TrapezoidProfile.Constraints autoBalanceConstaints = new TrapezoidProfile.Constraints(
      0.1, 0.1);
  

    public static final double maxAutoSpeed = 1.4;
    public static final double maxAutoAcceleration = 2.0;

    public static final HashMap<String, Command> eventMap = new HashMap<>();

    public static final double slowIntakeSpeed = 0.5;
    public static final double fastIntakeSpeed = 1;




  }
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class PIDSwerveConstants
  {
    public static final ProfiledPIDController thetaController = new ProfiledPIDController(0.04, 0, 0, AutoConstants.kThetaControllerConstraints);
    public static final ProfiledPIDController m_XController = new ProfiledPIDController(0.04, 0, 0, AutoConstants.kDriveControllerConstraints);
    public static final ProfiledPIDController m_YController = new ProfiledPIDController(0.1, 0, 0, AutoConstants.kDriveControllerConstraints);
    public static final ProfiledPIDController m_YAutoController = new ProfiledPIDController(0.001, 0, 0, AutoConstants.kDriveControllerConstraints);

  

  }

  public static final class TestMotorConstants {
    public static final int kTestMotor1CANID = 10;  //right trigger
    public static final int kTestMotor2CANID = 11;  //left trigger
    public static final int kTestMotor1FlexCANID = 12;  //left trigger WIP
    public static final int kTestMotor2FlexCANID = 13;  //right trigger

    public static final double kTestMotor1MaxSpeed = 0.10;
    public static final double kTestMotor2MaxSpeed = 0.10;
    public static final double kTestMotor1FlexMaxSpeed = 0.90; // WIP
    public static final double kTestMotor2FlexMaxSpeed = 0.90;
  
  }
  

  public static final class PneumaticsConstants {
    public static final int kPneumaticsModuleCANID = 20; 
    public static final int kSolenoidForwardChannel = 0; //TODO: check if needs to be swaped
    public static final int kSolenoidReverseChannel = 1; 
  }
}
