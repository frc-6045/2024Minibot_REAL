package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

//FIXME: this rn will only work with blue alliance, needs updating for red alliance
public class PoseMath {
    public static double FindShootingAngle(Pose2d pose){ 
        double angle;
        System.out.println(pose.getTranslation());
        double distanceToSpeakerBack = pose.getTranslation().getDistance(FieldConstants.kSpeakerBackLocation.getTranslation());
        double distanceToSpeakerFront = pose.getTranslation().getDistance(FieldConstants.kSpeakerFrontLoation.getTranslation());
        System.out.println(distanceToSpeakerBack + " " + distanceToSpeakerFront);
        double angleToBackLow = Math.toDegrees(Math.atan(FieldConstants.kLowSpeakerOpeningHeight/distanceToSpeakerBack));
        double angleToFrontHigh = Math.toDegrees(Math.atan(FieldConstants.kHighSpeakerOpeningHeight/distanceToSpeakerFront)); 
        System.out.println(angleToBackLow + " " +  angleToFrontHigh);
        angle = (angleToBackLow + angleToFrontHigh) / 2;
        System.out.println("shooting angle: " + angle);
        return angle;
    }

    public static double FindTurningAngle(Pose2d pose){
        double angle;
        double x = FieldConstants.kSpeakerBackLocation.getX() - pose.getX();
        double y = FieldConstants.kSpeakerBackLocation.getY() - pose.getY();
        angle = Math.toDegrees(-Math.atan2(x, y));
        if(angle < 0){
            angle = -angle + 180;
        }
        return angle;
    }

    public static Rotation2d getTargetAngle(Translation2d point, Pose2d currentPose){
        
        Rotation2d targetAngle = new Rotation2d(point.getX() - currentPose.getX(), point.getY() - currentPose.getY());
        if(currentPose.getRotation().getDegrees() < 0){
            return targetAngle.minus(targetAngle.times(2));
        }
        return targetAngle;
    }




    public static double getDistanceToSpeakerBack(Pose2d pose) {
        return pose.getTranslation().getDistance(FieldConstants.kSpeakerBackLocation.getTranslation());
    }


}
