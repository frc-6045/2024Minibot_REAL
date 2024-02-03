// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.math.interpolation.InterpolatingTreeMap;


/** Add your docs here. */
public class LookupTables {
    private static InterpolatingTreeMap<Double, Double> shooterAngleTable = new InterpolatingTreeMap<Double, Double>(
    (up, q, down) -> {
        double upperToLower = up.doubleValue() - down.doubleValue();
        if(upperToLower <= 0.0){
            return 0.0;
        }
        double queryToLower = q.doubleValue() - down.doubleValue();
        if(queryToLower <= 0.0){
            return 0.0;
        }
        return queryToLower/upperToLower;
    },
    (val1, val2, d) -> {
        double dydx = val2.doubleValue() - val1.doubleValue();
        return dydx * d + val1.doubleValue();
    });
    private static InterpolatingTreeMap<Double, Double> shooterSpeedTable = new InterpolatingTreeMap<Double, Double>(
    (up, q, down) -> {
        double upperToLower = up.doubleValue() - down.doubleValue();
        if(upperToLower <= 0.0){
            return 0.0;
        }
        double queryToLower = q.doubleValue() - down.doubleValue();
        if(queryToLower <= 0.0){
            return 0.0;
        }
        return queryToLower/upperToLower;
    },
    (val1, val2, d) -> {
        double dydx = val2.doubleValue() - val1.doubleValue();
        return dydx * d + val1.doubleValue();
    });



    /**
     * <pre>
     * Inits Values for the Interpolating Tables
     * Key is Distance from Speaker,
     * Value is RPM or Angle Value at that distance
     * </pre>
     */
    public static void InitValues() {
        //trash values!!!!!!!!

        
        shooterAngleTable.put(0.0, 5e3);
        shooterSpeedTable.put(0.0, 5e3);
    }

    public static InterpolatingTreeMap<Double, Double> getAngleTable(){
        InitValues(); 
        return shooterAngleTable;
    }

    public static InterpolatingTreeMap<Double, Double> getSpeedTable(){
        InitValues();
        return shooterAngleTable;
    }

    public static double getSpeedValueAtDistance(double distMeterFromSpeaker) {
        return shooterSpeedTable.get(distMeterFromSpeaker);
    }

    public static double getAngleValueAtDistance(double distMeterFromSpeaker) {
        return shooterAngleTable.get(distMeterFromSpeaker);
    }
    
    
}
