package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    /* MOTORS */
    public TalonFX mRunningMotor;
    public TalonFX mAngleMotor;

    public Gearbox mGearbox = IntakeConstants.gearbox;

    /* SENSORS */
    public DutyCycleEncoder mBoreEncoder;
    public DigitalInput mBeamBreakSensor;

    /* ANGLE MOTOR CONTROL */
    private Position mPosition;
    private double mPositionSetpoint;
    private PositionVoltage mPositionControl;
    private double mAngleOpenLoopOutput;
    private DutyCycleOut mAngleOpenLoopControl;

    /* RUNNING/INTAKING MOTOR CONTROL */
    private Running mRunning;
    private double mRunningVelocitySetpoint;
    private VelocityVoltage mRunningVelocityControl;
    private double mRunningOpenLoopOutput;
    private DutyCycleOut mRunningOpenLoopControl;

    public IntakeSubsystem() {
        mRunningMotor = new TalonFX(IntakeConstants.runningMotorId, Constants.RIO_CANBUS);
        mAngleMotor = new TalonFX(IntakeConstants.angleMotorId, Constants.RIO_CANBUS);

        mBoreEncoder = new DutyCycleEncoder(new DigitalInput(IntakeConstants.boreEncoderDioId));
        mBeamBreakSensor = new DigitalInput(IntakeConstants.beamBreakSensorDioId);

        mPosition = Position.CLOSED;
        mRunning = Running.NEUTRAL;

        /* mAngleMotor + PositionControl setup */
        // TODO: fix PID constants
        var angleMotorConfigs = new Slot0Configs();
        angleMotorConfigs.kP = IntakeConstants.ANGLE_kP;
        angleMotorConfigs.kI = IntakeConstants.ANGLE_kI;
        angleMotorConfigs.kD = IntakeConstants.ANGLE_kD;

        mAngleMotor.getConfigurator().apply(angleMotorConfigs);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        // TODO: mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset()
        // must output a position relative to the horizontal, which is 0
        resetToAbsolute();
        mPositionSetpoint = mAngleMotor.getPosition().getValueAsDouble();
        mPositionControl = new PositionVoltage(mPositionSetpoint);

        /* mRunningMotor + VelocityControl setup */
        // TODO: fix PID constants
        var runningMotorConfigs = new Slot0Configs();
        runningMotorConfigs.kP = IntakeConstants.RUN_kP;
        runningMotorConfigs.kI = IntakeConstants.RUN_kI;
        runningMotorConfigs.kD = IntakeConstants.RUN_kD;

        mRunningMotor.getConfigurator().apply(runningMotorConfigs);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        mRunningVelocitySetpoint = 0;
        mRunningVelocityControl = new VelocityVoltage(mRunningVelocitySetpoint);
    }

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }

    public enum Position {
        OPEN,
        CLOSED,
        /** custom setpoint/position */
        OVERRIDE,
        /** openloop control */
        OPENLOOP,
    }

    public enum Running {
        /** intake */
        FORWARD,
        /** outtake */
        REVERSE,
        /** neutral/idle (brake) */
        NEUTRAL,
        /** custom setpoint/position */
        OVERRIDE,
        /** openloop control */
        OPENLOOP,
    }

    @Override
    public void periodic() {
        if (mBeamBreakSensor.get()) {
            setRunningState(Running.NEUTRAL);
        }

        // TODO: Control requests are automatically transmitted at a fixed update
        // frequency.
        // does this mean that we do not have to setControl every periodic run?
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/control-requests.html#changing-update-frequency
        // nevertheless I added them here just to be sure
        switch (mPosition) {
            case OPENLOOP:
                mAngleMotor.setControl(mAngleOpenLoopControl);
                break;

            default:
                /* GRAVITY FF */
                mAngleMotor.setControl(mPositionControl.withFeedForward(
                        IntakeConstants.kG * Math.cos(Conversions.revolutionsToRadians(getBoreEncoderPosition()))));
                break;
        }

        switch (mRunning) {
            case OPENLOOP:
                mRunningMotor.setControl(mRunningOpenLoopControl);
                break;

            case NEUTRAL:
                mRunningMotor.stopMotor();

            default:
                mRunningMotor.setControl(mRunningVelocityControl);
                break;
        }

        // STOP ANGLE MOTOR IF WE ARE GOING INTO THE DRIVEBASE
        if (getBoreEncoderPosition() < IntakeConstants.stopPosition) {
            setAngleOpenLoop(0);
            mAngleMotor.stopMotor();
            resetToAbsolute();
        }

    }

    /* CALCULATIONS */
    public double drivenToDriver(double revolutions) {
        return revolutions * (1.0 / mGearbox.getRatio());
    }

    public double driverToDriven(double revolutions) {
        return mGearbox.calculate(revolutions);
    }

    /* ENCODERS */
    // theoretically, Falcon position / 72 = Bore encoder position + offset at all
    // times

    public double getFalconPosition() {
        return mAngleMotor.getPosition().getValueAsDouble();
    }

    /** Bore encoder reading (with position offset) */
    public double getBoreEncoderPosition() {
        return mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset;
    }

    public void resetToAbsolute() {
        double position = drivenToDriver(mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset);
        mAngleMotor.setPosition(position);
    }

    /* SETPOINT CHECKS */

    public boolean isAtPositionSetpoint() {
        if (mPosition == Position.OPENLOOP) {
            return false;
        }
        return Math.abs(mAngleMotor.getPosition().getValueAsDouble()
                - mPositionSetpoint) < IntakeConstants.positionEqualityTolerance;
    }

    public boolean isAtVelocitySetpoint() {
        if (mRunning == Running.OPENLOOP) {
            return false;
        }
        return Math.abs(mRunningMotor.getPosition().getValueAsDouble()
                - mRunningVelocitySetpoint) < IntakeConstants.velocityEqualityTolerance;
    }

    /* STATE ACCESSORS */

    public Position getIntakePosition() {
        return mPosition;
    }

    public Running getIntakeRunning() {
        return mRunning;
    }

    /* STATE SETTING */

    public void setStates(Position pos, Running run) {
        setRunningState(run);
        setPositionState(pos);
    }

    public void setRunningState(Running run) {
        mRunning = run;

        switch (mRunning) {
            case FORWARD:
                mRunningVelocitySetpoint = IntakeConstants.forwardVelocity;
                break;

            case REVERSE:
                mRunningVelocitySetpoint = IntakeConstants.reverseVelocity;
                break;

            case NEUTRAL:
                mRunningVelocitySetpoint = 0;
                mRunningMotor.stopMotor();
                return;

            case OVERRIDE:
                break;

            case OPENLOOP:
                // don't set control if it's open loop, just return
                return;
        }

        mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
        mRunningMotor.setControl(mRunningVelocityControl);
    }

    public void setRunningOverride(double velocity) {
        mRunning = Running.OVERRIDE;
        mRunningVelocitySetpoint = velocity;
        mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
        mRunningMotor.setControl(mRunningVelocityControl);
    }

    public void setRunningOpenLoop(double output) {
        mRunning = Running.OPENLOOP;
        mRunningOpenLoopOutput = output;
        mRunningOpenLoopControl.Output = mRunningOpenLoopOutput;
        mRunningMotor.setControl(mRunningOpenLoopControl);
    }

    public void setPositionState(Position pos) {
        mPosition = pos;

        switch (mPosition) {
            case OPEN:
                mPositionSetpoint = drivenToDriver(IntakeConstants.openPosition);
                break;

            case CLOSED:
                mPositionSetpoint = drivenToDriver(IntakeConstants.openPosition);
                break;

            case OVERRIDE:
                break;

            case OPENLOOP:
                break;
        }

        mPositionControl.Position = mPositionSetpoint;
        mAngleMotor.setControl(mPositionControl);
    }

    public void setPositionOverride(double position) {
        mPosition = Position.OVERRIDE;
        mPositionSetpoint = position;
        mPositionControl.Position = mPositionSetpoint;
        mAngleMotor.setControl(mPositionControl);
    }

    public void setAngleOpenLoop(double output) {
        mPosition = Position.OPENLOOP;
        mAngleOpenLoopOutput = output;
        mAngleOpenLoopControl.Output = mAngleOpenLoopOutput;
        mAngleMotor.setControl(mAngleOpenLoopControl);
    }

    public ArrayList<TalonFX> getMotors() {
        var motors = new ArrayList<TalonFX>();
        motors.add(mRunningMotor);
        motors.add(mAngleMotor);
        return motors;
    }
}
