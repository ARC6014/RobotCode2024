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

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.SwerveUtils.CTREConfigs;
import frc.team6014.lib.util.SwerveUtils.CTREModuleState;
import frc.team6014.lib.util.SwerveUtils.SwerveDriveConstants;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;

/** Base for constructing any swerve module
 * * Basic Logic is that:
 * * we have 3 fields: xspd, yspd, rot
 * * 1) convert the speeds to the robot's perspective --> ChassisSpeeds
 * * 2) convert the chassis speeds to the individual module's perspective --> SwerveModuleState
 * * 3) optimize the module states to minimize the change in heading --> CTREModuleState
 * * 4) apply the states to the modules --> setDesiredState (done with FF and PID) 
 */
public class SwerveModuleBase {

    private String mId;
    private int mModuleNumber;
    private WPI_TalonFX mDriveMotor;
    private WPI_TalonFX mAngleMotor;
    private WPI_CANCoder mRotEncoder;

    private SimpleMotorFeedforward mDriveFeedforward;

    private boolean isDriveMotorInverted = false; // TODO: Check all
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

    public SwerveModuleBase(int moduleNum, String name, SwerveModuleConstants constants, SwerveDriveConstants swerveConstants) {

        mId = name;

        mModuleNumber = moduleNum;

        kAngleOffset = constants.CANCoderAngleOffset;


        mDriveFeedforward = new SimpleMotorFeedforward(constants.moduleTuningkS, constants.moduleTuningkV, DriveConstants.drivekA);

        mRotEncoder = new WPI_CANCoder(constants.CANCoderID, Constants.CANIVORE_CANBUS);
        mDriveMotor = new WPI_TalonFX(constants.driveMotorID, Constants.CANIVORE_CANBUS);
        mAngleMotor = new WPI_TalonFX(constants.angleMotorID, Constants.CANIVORE_CANBUS);

        configAll(); // Configs all the motors and encoders

        mRotEncoder.configMagnetOffset(constants.CANCoderAngleOffset);

        resetToAbsolute();

        lastAngle = getCANCoderRotation().getDegrees();

    }

    /*
     * Setters
     */

    public void stop() {
        mDriveMotor.set(.0);
        mAngleMotor.set(.0);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle); // minimize the change in heading/easiest way

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / maxSpeed;
            mDriveMotor.set(TalonFXControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, mWheelCircumference,
                    driveGearbox.getRatio());
            mDriveMotor.set(TalonFXControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
                    mDriveFeedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle.getDegrees();

        mAngleMotor.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(lastAngle, angleGearbox.getRatio()));

        lastAngle = angle;

    }

    public void setNeutralMode2Brake(boolean brake) {
        mDriveMotor.setNeutralMode(brake ? NeutralMode.Brake : NeutralMode.Coast);
    }

    /*
     * Configs
     */

    private void configRotEncoder() {
        mRotEncoder.configFactoryDefault();
        mRotEncoder = CTREConfigs.swerveCancoderConfig(mRotEncoder);
        mRotEncoder.configSensorDirection(isRotEncoderInverted);
    }

    private void configAngleMotor() {
        mAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        mAngleMotor = CTREConfigs.swerveAngleFXConfig(mAngleMotor);
        mAngleMotor.setInverted(isAngleMotorInverted);
    }

    private void configDriveMotor() {
        mDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 20);
        mDriveMotor = CTREConfigs.swerveDriveFXConfig(mDriveMotor);
        mDriveMotor.setInverted(isDriveMotorInverted);
    }

    public void configAll() {
        configRotEncoder();
        configDriveMotor();
        configAngleMotor();
    }

    public void resetToAbsolute() {
        double position = Conversions.degreesToFalcon(getCANCoderRotation().getDegrees(), angleGearbox.getRatio());
        mAngleMotor.setSelectedSensorPosition(position);
    }

    /*
     * Getters
     */

    public int getModuleNumber() {
        return mModuleNumber;
    }

    public WPI_TalonFX getDriveMotor() {
        return mDriveMotor;
    }

    // public double getDriveMotorCurrent() {
    //     return mDriveMotor.getStatorCurrent().getValue();
    // }

    public WPI_TalonFX getAngleMotor() {
        return mAngleMotor;
    }

    public double getAngleOffset() {
        return kAngleOffset;
    }

    public String getName() {
        return mId;
    }

    public Rotation2d getCANCoderRotation() {
        return Rotation2d.fromDegrees(mRotEncoder.getAbsolutePosition());
    }

    public SwerveModulePosition getPosition(){
        double position = Conversions.falconToRotation(mDriveMotor.getSelectedSensorPosition(), driveGearbox.getRatio()) * mWheelCircumference;
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearbox.getRatio()));
        return new SwerveModulePosition(position, angle);
    }

    public SwerveModuleState getState() {
        double velocity = Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), mWheelCircumference, driveGearbox.getRatio());
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), angleGearbox.getRatio()));
        return new SwerveModuleState(velocity, angle);
    }


}
