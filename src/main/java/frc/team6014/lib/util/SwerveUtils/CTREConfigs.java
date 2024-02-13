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

    public static TalonFX swerveDriveFXConfig(TalonFX talon) {
        // TODO: this feature is under "Features Omitted" in CTRE v6 docs
        // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/feature-replacements-guide.html#features-omitted
        // talon.configAllowableClosedloopError(0, 0, 20);
        talon.getConfigurator().apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(DriveConstants.driveEnableCurrentLimit)
                .withSupplyCurrentLimit(DriveConstants.driveContinuousCurrentLimit)
                .withSupplyCurrentThreshold(DriveConstants.drivePeakCurrentLimit)
                .withSupplyTimeThreshold(DriveConstants.drivePeakCurrentDuration)
        );
        talon.getConfigurator().apply(
            new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(DriveConstants.openLoopRamp)
                .withVoltageClosedLoopRampPeriod(DriveConstants.closedLoopRamp)
                .withTorqueClosedLoopRampPeriod(DriveConstants.closedLoopRamp)
        );
        talon.setNeutralMode(DriveConstants.driveMotorNeutralMode == NeutralMode.Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        talon.getConfigurator().apply(
            new Slot0Configs()
                .withKP(V6Converter.v5VelocityWithoutVoltageCompensation_kP(DriveConstants.drivekP))
                .withKI(V6Converter.v5VelocityWithoutVoltageCompensation_kI(DriveConstants.drivekI))
                .withKD(V6Converter.v5VelocityWithoutVoltageCompensation_kD(DriveConstants.drivekD))
        );
        return talon;
    }

    public static TalonFX swerveAngleFXConfig(TalonFX talon) {
        talon.getConfigurator().apply(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(DriveConstants.angleEnableCurrentLimit)
                .withSupplyCurrentLimit(DriveConstants.angleContinuousCurrentLimit)
                .withSupplyCurrentThreshold(DriveConstants.anglePeakCurrentLimit)
                .withSupplyTimeThreshold(DriveConstants.anglePeakCurrentDuration)
        );
        talon.getConfigurator().apply(
            new ClosedLoopRampsConfigs()
                .withDutyCycleClosedLoopRampPeriod(DriveConstants.openLoopRamp)
                .withVoltageClosedLoopRampPeriod(DriveConstants.closedLoopRamp)
                .withTorqueClosedLoopRampPeriod(DriveConstants.closedLoopRamp)
        );
        talon.setNeutralMode(DriveConstants.angleMotorNeutralMode == NeutralMode.Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        talon.getConfigurator().apply(
            new Slot0Configs()
                .withKP(V6Converter.v5PositionWithoutVoltageCompensation_kP(DriveConstants.anglekP))
                .withKI(V6Converter.v5PositionWithoutVoltageCompensation_kI(DriveConstants.anglekI))
                .withKD(V6Converter.v5PositionWithoutVoltageCompensation_kD(DriveConstants.anglekD))
        );
        return talon;
    }

    public static CANcoder swerveCancoderConfig(CANcoder CANcoder) {
        CANcoder.getConfigurator().apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    )
        );
        return CANcoder;
    }
}
