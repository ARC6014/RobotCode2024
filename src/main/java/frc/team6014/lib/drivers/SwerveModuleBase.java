// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6014.lib.drivers;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.SwerveUtils.CTREConfigs;
import frc.team6014.lib.util.SwerveUtils.CTREModuleState;
import frc.team6014.lib.util.SwerveUtils.SwerveDriveConstants;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;
import io.github.oblarg.oblog.Loggable;

/**
 * Base for constructing any swerve module
 * * Basic Logic is that:
 * * we have 3 fields: xspd, yspd, rot
 * * 1) convert the speeds to the robot's perspective --> ChassisSpeeds
 * * 2) convert the chassis speeds to the individual module's perspective -->
 * SwerveModuleState
 * * 3) optimize the module states to minimize the change in heading -->
 * CTREModuleState
 * * 4) apply the states to the modules --> setDesiredState (done with FF and
 * PID)
 */
public class SwerveModuleBase implements Loggable {

    private String mId;
    private int mModuleNumber;
    private TalonFX mDriveMotor6;
    private TalonFX mAngleMotor6;
    private CANcoder mRotEncoder6;

    private SimpleMotorFeedforward mDriveFeedforward;

    private boolean isDriveMotorInverted = false; 
    private boolean isAngleMotorInverted = true;
    private boolean isRotEncoderInverted = false;

    private Gearbox driveGearbox = new Gearbox(DriveConstants.driveGearboxRatio);
    private Gearbox angleGearbox = new Gearbox(DriveConstants.angleGearboxRatio);

    private double mWheelCircumference = DriveConstants.wheelCircumference;

    private double maxSpeed = DriveConstants.maxSpeed;

    private final double kAngleOffset;
    private double lastAngle;

    /**
     * @param moduleNum       Number of the module
     * @param name            Name of the module
     * @param constants       Individual constants for a module
     * @param swerveConstants Global swerve base constants
     */

    public SwerveModuleBase(int moduleNum, String name, SwerveModuleConstants constants,
            SwerveDriveConstants swerveConstants) {

        mId = name;

        mModuleNumber = moduleNum;

        kAngleOffset = constants.CANCoderAngleOffset;

        mDriveFeedforward = new SimpleMotorFeedforward(constants.moduleTuningkS, constants.moduleTuningkV,
                DriveConstants.drivekA);

        mRotEncoder6 = new CANcoder(constants.CANCoderID, Constants.CANIVORE_CANBUS);
        mDriveMotor6 = new TalonFX(constants.driveMotorID, Constants.CANIVORE_CANBUS);
        mAngleMotor6 = new TalonFX(constants.angleMotorID, Constants.CANIVORE_CANBUS);

        configAll(); // Configs all the motors and encoders

        resetToAbsolute();

        lastAngle = getCANCoderRotation().getDegrees();

    }

    /*
     * Setters
     */

    public void stop() {
        mDriveMotor6.set(0);
        mAngleMotor6.set(0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // minimize the change in
                                                                                     // heading/easiest way
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor6.setControl(new DutyCycleOut(percentOutput));
        } else {
            double velocity = desiredState.speedMetersPerSecond / mWheelCircumference;
            mDriveMotor6.setControl(new VelocityDutyCycle(velocity).withFeedForward(mDriveFeedforward.calculate(desiredState.speedMetersPerSecond)));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();

        mAngleMotor6.setControl(new PositionDutyCycle(lastAngle / 360.0));

        lastAngle = angle;

    }

    public double getVelocityMPS() {
        return mDriveMotor6.getVelocity().getValueAsDouble() * mWheelCircumference;
    }

    public void setNeutralMode2Brake(boolean brake) {
        mDriveMotor6.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    /*
     * Configs
     */

    // V5 to V6: there is no need to reset to factory defaults as the Configuration object
    // sets to factory defaults unless edited

    private void configRotEncoder() {
        CANcoderConfiguration config = CTREConfigs.swerveCANCoderConfig();
        config.MagnetSensor.SensorDirection = isRotEncoderInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = kAngleOffset / 360;
        mRotEncoder6.getConfigurator().apply(config);
    }

    private void configAngleMotor() {
        mAngleMotor6.setNeutralMode(DriveConstants.angleMotorNeutralMode == NeutralMode.Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        TalonFXConfiguration config = CTREConfigs.swerveAngleConfig();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.MotorOutput.Inverted = isAngleMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = angleGearbox.getRatio();
        mAngleMotor6.getConfigurator().apply(config);
    }

    private void configDriveMotor() {
        mDriveMotor6.setNeutralMode(DriveConstants.driveMotorNeutralMode == NeutralMode.Coast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
        TalonFXConfiguration config = CTREConfigs.swerveAngleConfig();
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.MotorOutput.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = driveGearbox.getRatio();
        mAngleMotor6.getConfigurator().apply(config);
    }

    public void configAll() {
        configRotEncoder();
        configDriveMotor();
        configAngleMotor();
    }

    public void resetToAbsolute() {
        double position6 = getCANCoderRotation().getRotations();
        mAngleMotor6.setPosition(position6);
    }

    /*
     * Getters
     */

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public TalonFX getDriveMotor() {
        return mDriveMotor6;
    }

    // public double getDriveMotorCurrent() {
    // return mDriveMotor.getStatorCurrent().getValue();
    // }

    public TalonFX getAngleMotor() {
        return mAngleMotor6;
    }

    public double getAngleOffset() {
        return kAngleOffset;
    }

    public String getName() {
        return mId;
    }

    public Rotation2d getCANCoderRotation() {
        return Rotation2d.fromRotations(mRotEncoder6.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModulePosition getPosition() {
        double position = mDriveMotor6.getPosition().getValueAsDouble() * mWheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor6.getPosition().getValueAsDouble());
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = mDriveMotor6.getVelocity().getValueAsDouble() * mWheelCircumference;
        Rotation2d angle = Rotation2d.fromRotations(mAngleMotor6.getPosition().getValueAsDouble());
        return new SwerveModuleState(velocity, angle);
    }

}
