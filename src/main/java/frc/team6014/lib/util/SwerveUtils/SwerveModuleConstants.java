// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.util.SwerveUtils;

/**
 * Constants for individual swerve modules
 */
public class SwerveModuleConstants {
    public final int driveMotorID;
    public final int angleMotorID;
    public final int CANCoderID;
    public final double CANCoderAngleOffset;
    public final double moduleTuningkS;
    public final double moduleTuningkV;

    public SwerveModuleConstants(int driveMotorID, int angleMotorID, int CANCoderID, double CANCoderAngleOffset,
            double moduleTuningkS, double moduleTuningkV) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.CANCoderID = CANCoderID;
        this.CANCoderAngleOffset = CANCoderAngleOffset;
        this.moduleTuningkS = moduleTuningkS;
        this.moduleTuningkV = moduleTuningkV;
    }

    public static SwerveModuleConstants generateModuleConstants(int driveMotorID, int angleMotorID, int CANCoderID,
            double angleOffset, double moduleTuningkS, double moduleTuningkV) {
        return new SwerveModuleConstants(driveMotorID, angleMotorID, CANCoderID, angleOffset, moduleTuningkS,
                moduleTuningkV);
    }
}
