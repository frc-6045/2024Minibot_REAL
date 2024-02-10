// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;


import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;


/** Add your docs here. */
public class LookupTables {
    private static InterpolatingTreeMap<Double, Double> shooterAngleTable = new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    private static InterpolatingTreeMap<Double, Double> shooterSpeedTable = new InterpolatingTreeMap<Double, Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    
    // private static InterpolatingTreeMap<Double, Double> shooterAngleTable = new InterpolatingTreeMap<Double, Double>(
    // (up, q, down) -> {
    //     double upperToLower = up.doubleValue() - down.doubleValue();
    //     if(upperToLower <= 0.0){
    //         return 0.0;
    //     }
    //     double queryToLower = q.doubleValue() - down.doubleValue();
    //     if(queryToLower <= 0.0){
    //         return 0.0;
    //     }
    //     return queryToLower/upperToLower;
    // },
    // (val1, val2, d) -> {
    //     double dydx = val2.doubleValue() - val1.doubleValue();
    //     return dydx * d + val1.doubleValue();
    // });
    // private static InterpolatingTreeMap<Double, Double> shooterSpeedTable = new InterpolatingTreeMap<Double, Double>(
    // (up, q, down) -> {
    //     double upperToLower = up.doubleValue() - down.doubleValue();
    //     if(upperToLower <= 0.0){
    //         return 0.0;
    //     }
    //     double queryToLower = q.doubleValue() - down.doubleValue();
    //     if(queryToLower <= 0.0){
    //         return 0.0;
    //     }
    //     return queryToLower/upperToLower;
    // },
    // (val1, val2, d) -> {
    //     double dydx = val2.doubleValue() - val1.doubleValue();
    //     return dydx * d + val1.doubleValue();
    // });



    /**
     * <pre>
     *Inits Values for the Interpolating Tables
     *Key is Distance from Speaker,
     *Value is RPM or Angle Value at that distance
     * </pre>
     */
    public static void InitValues() {
        //trash values!!!!!!!!
        shooterAngleTable.put(1.3716, 37.1728);
        shooterAngleTable.put(1.64465, 36.1939);
        shooterAngleTable.put(1.9812, 34.165);
        shooterAngleTable.put(2.4257, 30.163);
        shooterAngleTable.put(2.794, 27.1463);
        shooterAngleTable.put(3.2639, 26.121);
        shooterAngleTable.put(3.77825, 24.1629);
        shooterAngleTable.put(4.17195, 22.7821);
        shooterAngleTable.put(4.6736, 22.5978);


        shooterSpeedTable.put(0.0, 0.0);
    }

    public static InterpolatingTreeMap<Double, Double> getAngleTable(){
        InitValues(); 
        return shooterAngleTable;
    }

    public static InterpolatingTreeMap<Double, Double> getSpeedTable(){
        InitValues();
        return shooterAngleTable;
    }

    public static double getSpeedValueAtDistance(Double distMeterFromSpeaker) {
        InitValues();
        return (double) shooterSpeedTable.get(distMeterFromSpeaker);
    }

    public static double getAngleValueAtDistance(Double distMeterFromSpeaker) {
        InitValues();
        //System.out.println(shooterAngleTable.get(distMeterFromSpeaker));
        return (double) shooterAngleTable.get(distMeterFromSpeaker);  
    }
    
    
}
