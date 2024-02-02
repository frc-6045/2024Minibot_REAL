package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

public class PoseMath {
    public static double FindShootingAngle(Pose2d pose){ //TODO: test
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
}
