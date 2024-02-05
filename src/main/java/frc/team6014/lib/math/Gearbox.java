// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.math;

public class Gearbox {
    private double m_gearRatio;
    private double m_drivingGear;
    private double m_drivenGear;

    public Gearbox(double drivingGear, double drivenGear) {
        m_gearRatio = drivenGear / drivingGear;
        m_drivenGear = drivenGear;
        m_drivingGear = drivingGear;
    }

    public Gearbox(double gearRatio) {
        m_gearRatio = gearRatio;
        m_drivenGear = m_gearRatio;
        m_drivingGear = 1;
    }

    public double calculate(double rot) {
        return rot * m_gearRatio;
    }

    public double getRatio() {
        return m_gearRatio;
    }

    public double drivenToDriving(double revolutions) {
      return (revolutions * m_drivenGear) / m_drivingGear;
    }

    public double drivingToDriven(double revolutions) {
      return (revolutions * m_drivingGear) / m_drivenGear;
    }
}
