// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.DriveSubsystem;


/** Add your docs here. */
public class Vision {
  private PhotonPoseEstimator m_visionPoseEstimator;
  private final DriveSubsystem m_drive;
  private Pose2d visionPose;
  public Vision(DriveSubsystem drive){
    try {
      m_visionPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new PhotonCamera("Arducam_OV2311_USB_Camera"), new Transform3d()); //TODO: actually make this work
    } catch(IOException e){
      System.out.println(e.getMessage() + "\n april tags didnt load");
    }
    m_drive = drive;
  } 

  public void UpdateVision() {
     m_visionPoseEstimator.update().ifPresent(estimatedRobotPose -> {
      System.out.println(estimatedRobotPose.estimatedPose.toPose2d().toString());
      
      m_drive.getPoseEstimator().addVisionMeasurement(estimatedRobotPose.estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
      visionPose = estimatedRobotPose.estimatedPose.toPose2d();
      //SmartDashboard.putNumber("Estimated Angle", PoseMath.FindShootingAngle(estimatedRobotPose.estimatedPose.toPose2d())); //enhnngg
      SmartDashboard.putNumber("estimated dist", PoseMath.getDistanceToSpeakerBack(estimatedRobotPose.estimatedPose.toPose2d()));
      SmartDashboard.putNumber("vision angle", visionPose.getRotation().getDegrees());
      SmartDashboard.putNumber("estimated proper turning angle", PoseMath.FindTurningAngle(estimatedRobotPose.estimatedPose.toPose2d()));
    });
  }

  public Pose2d getVisionPose() {
    return visionPose;
  }

  public double getDistance(){
    return PoseMath.getDistanceToSpeakerBack(visionPose);
  }
}
