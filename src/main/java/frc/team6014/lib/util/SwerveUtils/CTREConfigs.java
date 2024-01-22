// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.util.SwerveUtils;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import frc.robot.Constants.DriveConstants;

public class CTREConfigs {
    public static WPI_TalonFX swerveDriveFXConfig(WPI_TalonFX talon) {
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveConstants.driveEnableCurrentLimit, 
            DriveConstants.driveContinuousCurrentLimit, 
            DriveConstants.drivePeakCurrentLimit, 
            DriveConstants.drivePeakCurrentDuration);
        talon.configAllowableClosedloopError(0, 0, 20);
        talon.configSupplyCurrentLimit(driveSupplyLimit);
        talon.configOpenloopRamp(DriveConstants.openLoopRamp);
        talon.configClosedloopRamp(DriveConstants.closedLoopRamp);
        talon.setNeutralMode(DriveConstants.driveMotorNeutralMode);
        talon.config_kP(0, DriveConstants.drivekP);
        talon.config_kI(0, DriveConstants.drivekI);
        talon.config_kD(0, DriveConstants.drivekD);
        return talon;
    }

    public static WPI_TalonFX swerveAngleFXConfig(WPI_TalonFX talon) {
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveConstants.angleEnableCurrentLimit, 
            DriveConstants.angleContinuousCurrentLimit, 
            DriveConstants.anglePeakCurrentLimit, 
            DriveConstants.anglePeakCurrentDuration);
        talon.configAllowableClosedloopError(0, 0, 20);
        talon.configSupplyCurrentLimit(driveSupplyLimit);
        talon.configOpenloopRamp(DriveConstants.openLoopRamp);
        talon.configClosedloopRamp(DriveConstants.closedLoopRamp);
        talon.setNeutralMode(DriveConstants.angleMotorNeutralMode);
        talon.config_kP(0, DriveConstants.anglekP);
        talon.config_kI(0, DriveConstants.anglekI);
        talon.config_kD(0, DriveConstants.anglekD);
        return talon;
    }

    public static WPI_CANCoder swerveCancoderConfig(WPI_CANCoder CANcoder) { 
        CANcoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        CANcoder.configAllSettings(config);
        CANcoder.configGetFeedbackTimeBase(10);
        return CANcoder;
    }
}
