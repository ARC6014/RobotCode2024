// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.util.SwerveUtils;

/** 
 * Current limits for swerve drive/angle motors
 */
public class SwerveDriveConstants {
    public final int m_angleContinuousCurrentLimit;
    public final int m_anglePeakCurrentLimit;
    public final double m_anglePeakCurrentDuration;
    public final boolean m_angleEnableCurrentLimit;

    public final int m_driveContinuousCurrentLimit;
    public final int m_drivePeakCurrentLimit;
    public final double m_drivePeakCurrentDuration;
    public final boolean m_driveEnableCurrentLimit;

    public final double m_openLoopRamp;
    public final double m_closedLoopRamp;

    public SwerveDriveConstants(
            int angleContinuousCurrentLimit, int anglePeakCurrentLimit, double anglePeakCurrentDuration,
            boolean angleEnableCurrentLimit,
            int driveContinuousCurrentLimit, int drivePeakCurrentLimit, double drivePeakCurrentDuration,
            boolean driveEnableCurrentLimit,
            double openLoopRamp, double closedLoopRamp) {

        m_angleContinuousCurrentLimit = angleContinuousCurrentLimit;
        m_anglePeakCurrentLimit = anglePeakCurrentLimit;
        m_anglePeakCurrentDuration = anglePeakCurrentDuration;
        m_angleEnableCurrentLimit = angleEnableCurrentLimit;

        m_driveContinuousCurrentLimit = driveContinuousCurrentLimit;
        m_drivePeakCurrentLimit = drivePeakCurrentLimit;
        m_drivePeakCurrentDuration = drivePeakCurrentDuration;
        m_driveEnableCurrentLimit = driveEnableCurrentLimit;

        m_openLoopRamp = openLoopRamp;
        m_closedLoopRamp = closedLoopRamp;
    }

    public static SwerveDriveConstants generateSwerveConstants(
            int angleContinuousCurrentLimit, int anglePeakCurrentLimit, double anglePeakCurrentDuration,
            boolean angleEnableCurrentLimit,
            int driveContinuousCurrentLimit, int drivePeakCurrentLimit, double drivePeakCurrentDuration,
            boolean driveEnableCurrentLimit,
            double openLoopRamp, double closedLoopRamp) {

        return new SwerveDriveConstants(
                angleContinuousCurrentLimit, anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit,
                driveContinuousCurrentLimit, drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit,
                openLoopRamp, closedLoopRamp);
    }
}
