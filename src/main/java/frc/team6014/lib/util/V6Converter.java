package frc.team6014.lib.util;

public class V6Converter {
    // conversions from https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
    
    /// use PositionVoltage
    public static double v5PositionWithVoltageCompensation_kP(double v5_kP) {
        return v5PositionWithoutVoltageCompensation_kP(v5_kP) * 12;
    }

    /// use PositionVoltage
    public static double v5PositionWithVoltageCompensation_kI(double v5_kI) {
        return v5PositionWithoutVoltageCompensation_kI(v5_kI) * 12;
    }

    /// use PositionVoltage
    public static double v5PositionWithVoltageCompensation_kD(double v5_kD) {
        return v5PositionWithoutVoltageCompensation_kD(v5_kD) * 12;
    }

    /// use PositionDutyCycle
    public static double v5PositionWithoutVoltageCompensation_kP(double v5_kP) {
        return v5_kP * 2048 * (1/1023);
    }

    /// use PositionDutyCycle
    public static double v5PositionWithoutVoltageCompensation_kI(double v5_kI) {
        return v5_kI * 2048 * (1/1023) * 1000;
    }
    /// use PositionDutyCycle
    public static double v5PositionWithoutVoltageCompensation_kD(double v5_kD) {
        return v5_kD * 2048 * (1/1023) / 1000;
    }

    /// use VelocityVoltage
    public static double v5VelocityWithVoltageCompensation_kP(double v5_kP) {
        return v5VelocityWithoutVoltageCompensation_kP(v5_kP) * 12;
    }

    /// use VelocityVoltage
    public static double v5VelocityWithVoltageCompensation_kI(double v5_kI) {
        return v5VelocityWithoutVoltageCompensation_kI(v5_kI) * 12;
    }

    /// use VelocityVoltage
    public static double v5VelocityWithVoltageCompensation_kD(double v5_kD) {
        return v5VelocityWithoutVoltageCompensation_kD(v5_kD) * 12;
    }

    /// use VelocityVoltage and kF is now kV
    public static double v5VelocityWithVoltageCompensation_kF(double v5_kF) {
        return v5VelocityWithoutVoltageCompensation_kF(v5_kF) * 12;
    }

    /// use VelocityDutyCycle
    public static double v5VelocityWithoutVoltageCompensation_kP(double v5_kP) {
        return v5_kP * 2048 * (1/1023) * (1/10);
    }

    /// use VelocityDutyCycle
    public static double v5VelocityWithoutVoltageCompensation_kI(double v5_kI) {
        return v5_kI * 2048 * (1/1023) * 1000 * (1/10);
    }
    /// use VelocityDutyCycle
    public static double v5VelocityWithoutVoltageCompensation_kD(double v5_kD) {
        return v5_kD * 2048 * (1/1023) / 1000 * (1/10);
    }

    /// use VelocityDutyCycle and kF is now kV
    public static double v5VelocityWithoutVoltageCompensation_kF(double v5_kF) {
        return v5_kF * 2048 * (1/1023) * (1/10);
    }

}
