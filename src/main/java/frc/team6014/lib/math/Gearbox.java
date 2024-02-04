// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.math;

public class Gearbox {
    private double m_gearRatio;
    private double drivingGear;
    private double drivenGear;

    public Gearbox(double drivingGear, double drivenGear) {
        m_gearRatio = drivenGear / drivingGear;
    }

    public Gearbox(double gearRatio) {
        m_gearRatio = gearRatio;
    }

    public double calculate(double rot) {
        return rot * m_gearRatio;
    }

    public double getRatio() {
        return m_gearRatio;
    }

    public double drivenToDriving(double revolutions) {
      return revolutions / drivenGear * drivingGear;
    }

    public double drivingToDriven(double revolutions) {
      return revolutions / drivingGear * drivenGear;
    }
}
