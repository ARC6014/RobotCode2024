// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.math;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Inspired by team 254
 * Customized ARC CTRE configurations
 */
public class Conversions {
    /**
     * @param degrees   Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks = degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param counts    Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM       RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon
     *                  RPM)
     * @return RPM of Mechanism
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference  Circumference of Wheel
     * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for
     *                       Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity      Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }
    
    /**
     * @param counts    Falcon Sensor Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Rotation of the Mechanism
     */
    public static double falconToRotation(double counts, double gearRatio){
        double rotation = counts / 2048.0;
        return rotation / gearRatio;
    }

    /**
     * @param rotation Rotation of the Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Sensor Counts of Falcon
     */

    public static double rotationToFalcon(double rotation, double gearRatio){
        double counts = rotation * 2048.0;
        return counts * gearRatio;
    }

    /**
     * @param rad Rotation of the Mechanism in radians
     * @param gearing Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Sensor Counts of Falcon
     */

    public static double radiansToSteps(double rad,double gearing){
        double radPerStep = (((2*Math.PI)/gearing)) / 2048;
        return rad/radPerStep;
    }

    public static double revolutionsToRadians(double revolutions) {
        return revolutions * 2 * Math.PI;
    }

    public static double radiansToRevolutions(double radians) {
        return radians / (2 * Math.PI);
    }



    public static double convertAngleByAlliance(Alliance alliance, double angle) {
        angle = alliance == Alliance.Red ? angle + 180 : angle;

        if (angle >= 360) {
            angle -= 360;
        } else if (angle <= -360) {
            angle += 360;
        }

        return angle;
    }

    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
