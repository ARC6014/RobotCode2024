// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.util.SwerveUtils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.util.V6Converter;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class CTREConfigs implements Loggable {

    @Config
    int a;

    public static TalonFXConfiguration swerveDriveConfig() {
        // TODO: this feature is under "Features Omitted" in CTRE v6 docs
        // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/feature-replacements-guide.html#features-omitted
        // talon.configAllowableClosedloopError(0, 0, 20);

        // Inverts, Gear Ratio, Neutral Mode configured in SwerveModuleBase
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.driveEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveContinuousCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = DriveConstants.drivePeakCurrentLimit;
        config.CurrentLimits.SupplyTimeThreshold = DriveConstants.drivePeakCurrentDuration;

        /* Ramping */
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.openLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DriveConstants.closedLoopRamp;

        /* PID */
        config.Slot0.kP = V6Converter.v5VelocityWithoutVoltageCompensation_kP(DriveConstants.drivekP);
        config.Slot0.kI = V6Converter.v5VelocityWithoutVoltageCompensation_kI(DriveConstants.drivekI);
        config.Slot0.kD = V6Converter.v5VelocityWithoutVoltageCompensation_kD(DriveConstants.drivekD);
        
        return config;
    }
 
    public static TalonFXConfiguration swerveAngleConfig() {
        // Inverts, Gear Ratio, Neutral Mode configured in SwerveModuleBase
        TalonFXConfiguration config = new TalonFXConfiguration();
        /* Current Limiting */
        config.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.angleEnableCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimit = DriveConstants.angleContinuousCurrentLimit;
        config.CurrentLimits.SupplyCurrentThreshold = DriveConstants.anglePeakCurrentLimit;
        config.CurrentLimits.SupplyTimeThreshold = DriveConstants.anglePeakCurrentDuration;

        /* Ramping */
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DriveConstants.openLoopRamp;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.closedLoopRamp;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = DriveConstants.closedLoopRamp;

        /* PID */
        config.Slot0.kP = V6Converter.v5VelocityWithoutVoltageCompensation_kP(DriveConstants.anglekP);
        config.Slot0.kI = V6Converter.v5VelocityWithoutVoltageCompensation_kI(DriveConstants.anglekI);
        config.Slot0.kD = V6Converter.v5VelocityWithoutVoltageCompensation_kD(DriveConstants.anglekD);

        /* Continous wrap */
        config.ClosedLoopGeneral.ContinuousWrap = true;
        return config;
    }

    public static CANcoderConfiguration swerveCANCoderConfig() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        return config;
    }
}
