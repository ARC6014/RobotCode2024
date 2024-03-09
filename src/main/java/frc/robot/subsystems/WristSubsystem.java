package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class WristSubsystem extends SubsystemBase {
    private static WristSubsystem mInstance;

    /* MOTORS & ENCODER */
    private final TalonFX mTalonFX = new TalonFX(WristConstants.ANGLE_MOTOR_ID, Constants.CANIVORE_CANBUS);
    private final DutyCycleEncoder mBoreEncoder = new DutyCycleEncoder(WristConstants.BORE_ID);

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
    TalonFXConfiguration configs;
    private NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public WristSubsystem() {
        motorConfig();
        mBoreEncoder.setPositionOffset(WristConstants.POSITION_OFFSET);

        mPosition = Position.HOLD;

        m_timer.reset();
        m_timer.start();

        lastAbsoluteTime = m_timer.get();
    }

    public void motorConfig() {
        mTalonFX.getConfigurator().apply(new TalonFXConfiguration());
        configs = new TalonFXConfiguration();

        configs.Slot0.kP = WristConstants.ANGLE_kP;
        configs.Slot0.kI = WristConstants.ANGLE_kI;
        configs.Slot0.kD = WristConstants.ANGLE_kD;
        configs.Slot0.kS = WristConstants.ANGLE_kS;
        configs.Slot0.kV = WristConstants.ANGLE_kV;
        configs.MotorOutput.NeutralMode = kNeutralMode;

        configs.Voltage.PeakForwardVoltage = 12;
        configs.Voltage.PeakReverseVoltage = -12;
        configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
        configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;

        configs.CurrentLimits.SupplyCurrentLimitEnable = true;
        configs.CurrentLimits.SupplyCurrentLimit = 60;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;
        configs.CurrentLimits.StatorCurrentLimit = 60;

        configs.MotionMagic.MotionMagicAcceleration = WristConstants.WRIST_ACCELERATION;
        configs.MotionMagic.MotionMagicCruiseVelocity = WristConstants.WRIST_VELOCITY;

        mTalonFX.getConfigurator().apply(configs);
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

    public boolean isBoreEncoderAlive() {
        // return mBoreEncoder.isConnected();
        return false;
    }

    public double getAngle() {
        if (isBoreEncoderAlive()) {
            return getBoreEncoderPosition();
        } else {
            return getFalconPosition();
        }
    }

    public boolean isAtSetpoint() {
        if (isBoreEncoderAlive()) {
            return isAtSetpointBore();
        } else {
            return isAtSetpointFalcon();
        }
    }

    @Override
    public void periodic() {
        boolean shouldStopResetAccordingToBore = isBoreEncoderAlive() && isAtSetpointBore() && !isAtSetpointFalcon();

        if (shouldStopResetAccordingToBore) {
            mTalonFX.stopMotor();
            resetToAbsolute();
        }

        switch (mPosition) {
            case OPENLOOP:
                mTalonFX.setControl(mAngleOpenLoopControl);
                break;
            case OPEN:
                setWristAngleMotionMagic(WristConstants.OPEN_POSITION);
                break;
            case CLOSED:
                setWristAngleMotionMagic(WristConstants.CLOSED_POSITION);
                break;
            case HOLD:
                mTalonFX.setControl(new NeutralOut());
                break;
            default:
                setOpenLoop(0.0);
                break;
        }

        if (shouldStopResetAccordingToBore) {
            return;
        }

        // autoCalibration();
        SmartDashboard.putBoolean("Is Bore Connected Wrist", isBoreEncoderAlive());
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
        double boreAngle = mBoreEncoder.getAbsolutePosition() - mBoreEncoder.getPositionOffset();
        return boreAngle > 0 ? boreAngle : boreAngle + Conversions.degreesToRevolutions(360);
    }

    public void setWristAngleMotionMagic(double target) {
        mPositionSetpoint = target;
        mTalonFX.setControl(motionMagicVoltage.withPosition(
                mGearbox.drivenToDriving(Conversions.degreesToRevolutions(mPositionSetpoint))));
    }

    /** toggles neutral mode */
    public void setNeutralMode() {
        this.kNeutralMode = (kNeutralMode == NeutralModeValue.Brake) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        mTalonFX.setNeutralMode(this.kNeutralMode);
    }

    /** resets Falcon reading to absolute Bore reading */
    public void resetToAbsolute() {
        double angle = getBoreEncoderPosition();
        var position = mGearbox.drivenToDriving(angle);
        mTalonFX.setPosition(position);
        lastAbsoluteTime = m_timer.get();
    }

    public boolean isAtSetpointFalcon() {
        if (mPosition == Position.OPENLOOP) {
            return false;
        }
        return Math.abs(getFalconPosition()
                - Conversions.degreesToRevolutions(mPositionSetpoint)) < WristConstants.POSITION_EQUALITY_TOLERANCE;
    }

    public boolean isAtZero() {
        return getFalconPosition() < WristConstants.POSITION_EQUALITY_TOLERANCE;
    }

    public boolean isAtSetpointBore() {
        if (mPosition == Position.OPENLOOP) {
            return false;
        }
        return Math.abs(getBoreEncoderPosition()
                - Conversions.degreesToRevolutions(mPositionSetpoint)) < WristConstants.POSITION_EQUALITY_TOLERANCE;
    }

    /** @return setpoint unit: degrees */
    public double getSetpoint() {
        return mPositionSetpoint;
    }

    public Position getState() {
        return mPosition;
    }

    public void setState(Position pos) {
        mPosition = pos;
    }

    /** basically "zeroes" wrist */
    public double zeroSetpoint() {
        return mPositionSetpoint = 0;
    }

    public void resetFalconEncoder(double desiredAngle) {
        var desRot = Conversions.degreesToRevolutions(desiredAngle);
        mTalonFX.setPosition(desRot);
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
     * current Falcon rotation is 2 degrees off from the Bore reading
     * AND
     * the mechanism isn't moving
     */
    public void autoCalibration() {
        if (isBoreEncoderAlive()) {
            boolean timerCondition = m_timer.get() - lastAbsoluteTime > 10;
            boolean angleCondition = Math.abs(getBoreEncoderPosition() - getFalconPosition()) >= Conversions
                    .degreesToRevolutions(2);
            boolean speedCondition = Math.abs(mTalonFX.getRotorVelocity().getValueAsDouble()) < 0.005;
            if ((timerCondition || angleCondition) && speedCondition) {
                resetToAbsolute();
                lastAbsoluteTime = m_timer.get();
            }
        }
    }

    public TalonFX getMotor() {
        return mTalonFX;
    }

    public static WristSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new WristSubsystem();
        }
        return mInstance;
    }

}
