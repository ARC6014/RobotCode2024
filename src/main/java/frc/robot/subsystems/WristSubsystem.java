package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.ArmSubsystem.ArmControlState;
import frc.robot.Constants.WristConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class WristSubsystem extends SubsystemBase {
    private static WristSubsystem mInstance;

    /* MOTORS & ENCODER */
    private final TalonFX mTalonFX = new TalonFX(WristConstants.angleMotorId, Constants.CANIVORE_CANBUS);
    private final DutyCycleEncoder mBoreEncoder = new DutyCycleEncoder(WristConstants.boreEncoderDioId);

    private Gearbox mGearbox = WristConstants.gearbox;

    /** WRIST ANGLE POSITION */
    private Position mPosition;
    /** unit: degrees */
    // must output a position relative to the horizontal, which is 0
    private double mPositionSetpoint = 0;

    private final DutyCycleOut mAngleOpenLoopControl = new DutyCycleOut(0);
    private double mAngleOpenLoopOutput = 0;

    /** unit: rotations */
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

    /** Checking elapsed time for absolute calibration */
    private final Timer m_timer = new Timer();
    /** Last time when we resetted to absolute */
    private double lastAbsoluteTime;

    public WristSubsystem() {
        motorConfig();
        mBoreEncoder.setPositionOffset(WristConstants.positionOffset);

        mPosition = Position.HOLD;

        m_timer.reset();
        m_timer.start();

        lastAbsoluteTime = m_timer.get();
    }

    public void motorConfig() {
        mTalonFX.getConfigurator().apply(new TalonFXConfiguration());
        TalonFXConfiguration configs = new TalonFXConfiguration();

        // TODO: fix PID constants
        configs.Slot0.kP = WristConstants.ANGLE_kP;
        configs.Slot0.kI = WristConstants.ANGLE_kI;
        configs.Slot0.kD = WristConstants.ANGLE_kD;
        configs.Slot0.kS = WristConstants.ANGLE_kS;
        configs.Slot0.kV = WristConstants.ANGLE_kV;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;

        configs.MotionMagic.MotionMagicAcceleration = WristConstants.armAcceleration;
        configs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.armCruiseVelocity;

        mTalonFX.getConfigurator().apply(configs);
        mTalonFX.setNeutralMode(NeutralModeValue.Brake);
    }

    public static WristSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new WristSubsystem();
        }
        return mInstance;
    }

    public enum Position {
        /** ready for intaking */
        OPEN,
        /** closed to secure */
        CLOSED,
        /** custom setpoint/position */
        OVERRIDE,
        /** openloop control */
        OPENLOOP,
        /** for break mode in open loop (default) */
        HOLD,
    }

    @Override
    public void periodic() {
        switch (mPosition) {
            case OPENLOOP:
                mTalonFX.setControl(mAngleOpenLoopControl);
                break;
            case OPEN:
                setWristAngleMotionMagic(WristConstants.openPosition);
                break;
            case CLOSED:
                setWristAngleMotionMagic(WristConstants.closedPosition);
                break;
            case HOLD:
                mTalonFX.setControl(new NeutralOut());
                break;
            default:
                setOpenLoop(0.0);
                break;
        }

        // // STOP ANGLE MOTOR IF WE ARE GOING INTO THE DRIVEBASE
        // if (getBoreEncoderPosition() < WristConstants.stopPosition) {
        // setOpenLoop(0);
        // mTalonFX.stopMotor();
        // resetToAbsolute();
        // }

        //SmartDashboard.putNumber("Wrist Bore Reading", Conversions.revolutionsToDegrees(getBoreEncoderPosition()));
        //SmartDashboard.putNumber("Wrist Falcon Reading", Conversions.revolutionsToDegrees(getFalconPosition()));

        autoCalibration();
    }

    /* ENCODERS */
    // theoretically, Falcon position / 72 = Bore encoder position at all times
    /** unit: revolutions */
    public double getFalconPosition() {
        return mGearbox.drivingToDriven(mTalonFX.getPosition().getValueAsDouble());
    }

    /**
     * Bore encoder reading
     * unit: revolutions
     */
    public double getBoreEncoderPosition() {
        return mBoreEncoder.getAbsolutePosition() - mBoreEncoder.getPositionOffset();
    }

    // TODO: Add max/min angles for limit
    public void setWristAngleMotionMagic(double target) {
        mPositionSetpoint = target;
        mTalonFX.setControl(motionMagicVoltage.withPosition(
                mGearbox.drivenToDriving(Conversions.degreesToRevolutions(mPositionSetpoint))));
    }

    /** resets Falcon reading to absolute Bore reading */
    public void resetToAbsolute() {
        double angle = getBoreEncoderPosition();
        var position = mGearbox.drivenToDriving(angle);
        mTalonFX.setPosition(position);
        lastAbsoluteTime = m_timer.get();
    }

    public boolean isAtSetpoint() {
        if (mPosition == Position.OPENLOOP) {
            return false;
        }
        return Math.abs(getFalconPosition()
                - Conversions.degreesToRevolutions(mPositionSetpoint)) < WristConstants.positionEqualityTolerance;
    }

    public Position getState() {
        return mPosition;
    }

    public void setState(Position pos) {
        mPosition = pos;
    }

    /** @return setpoint unit: degrees */
    public double getSetpoint() {
        return mPositionSetpoint;
    }

    /** basically "zeroes" wrist */
    public double zeroSetpoint() {
        return mPositionSetpoint = 0;
    }

    public void setOverride(double position) {
        mPosition = Position.OVERRIDE;
        mPositionSetpoint = position;
        setWristAngleMotionMagic(position);
    }

    public void setOpenLoop(double output) {
        mPosition = Position.OPENLOOP;
        mAngleOpenLoopOutput = output;
        mTalonFX.setControl(mAngleOpenLoopControl.withOutput(output));
    }

    public void hold() {
        mPosition = Position.HOLD;
        mTalonFX.setControl(new NeutralOut());
    }

    /**
     * Resets to absolute if:
     * time has elapsed 10 seconds since previous calibration
     * OR
     * current Falcon rotation is 0.5 degrees off from the Bore reading
     * AND
     * the mechanism isn't moving
     */
    public void autoCalibration() {
        boolean timerCondition = m_timer.get() - lastAbsoluteTime > 10;
        boolean angleCondition = Math.abs(getBoreEncoderPosition() - getFalconPosition()) >= Conversions
                .degreesToRevolutions(0.5);
        boolean speedCondition = Math.abs(mTalonFX.getRotorVelocity().getValueAsDouble()) < 0.005;
        if ((timerCondition || angleCondition) && speedCondition) {
            resetToAbsolute();
            lastAbsoluteTime = m_timer.get();
        }
    }

    public ArrayList<TalonFX> getMotors() {
        var motors = new ArrayList<TalonFX>();
        motors.add(mTalonFX);
        return motors;
    }

}
