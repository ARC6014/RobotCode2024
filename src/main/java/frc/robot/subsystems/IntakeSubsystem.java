package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    /* MOTORS */
    public TalonFX mTalonFX;

    /* SENSORS */
    // public DigitalInput mBeamBreakSensor;

    /* RUNNING/INTAKING MOTOR CONTROL */
    private Running mRunning;
    /** unit: rps */
    private double mRunningVelocitySetpoint;
    private VelocityVoltage mRunningVelocityControl;

    private double mRunningOpenLoopOutput;
    private DutyCycleOut mRunningOpenLoopControl;

    public IntakeSubsystem() {
        mTalonFX = new TalonFX(IntakeConstants.runningMotorId, Constants.RIO_CANBUS);

        // mBeamBreakSensor = new DigitalInput(IntakeConstants.beamBreakSensorDioId);

        mRunning = Running.NEUTRAL;

        var motorConfig = new TalonFXConfiguration();
        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakForwardVoltage = -12;
        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        motorConfig.Slot0.kP = IntakeConstants.RUN_kP;
        motorConfig.Slot0.kI = IntakeConstants.RUN_kI;
        motorConfig.Slot0.kD = IntakeConstants.RUN_kD;
        motorConfig.Slot0.kS = IntakeConstants.RUN_kS;
        motorConfig.Slot0.kV = IntakeConstants.RUN_kV;

        mTalonFX.getConfigurator().apply(motorConfig);

        mRunningVelocitySetpoint = 0;
        mRunningVelocityControl = new VelocityVoltage(mRunningVelocitySetpoint);

        mRunningOpenLoopOutput = 0;
        mRunningOpenLoopControl = new DutyCycleOut(0);

        mTalonFX.setNeutralMode(NeutralModeValue.Brake);
    }

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
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
        /** testing when feeder is not assembled */
        FEEDING_SHOOTER,
    }

    @Override
    public void periodic() {
        // if (mBeamBreakSensor.get()) {
        // setState(Running.NEUTRAL);
        // }

        switch (mRunning) {
            case FORWARD:
                mRunningVelocitySetpoint = IntakeConstants.forwardVelocity;
                break;

            case REVERSE:
                mRunningVelocitySetpoint = IntakeConstants.reverseVelocity;
                break;

            case NEUTRAL:
                mRunningVelocitySetpoint = 0;
                mTalonFX.stopMotor();
                break;

            case OVERRIDE:
                break;

            case FEEDING_SHOOTER:
                mRunningVelocitySetpoint = IntakeConstants.feederVelocity;
                break;
            default:
                mRunningVelocitySetpoint = 0;
                break;
        }

        if (mRunning == Running.OPENLOOP)
            mTalonFX.setControl(mRunningOpenLoopControl);
        else {
            mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
            mTalonFX.setControl(mRunningVelocityControl);
        }
    }

    /* SETPOINT CHECKS */

    public boolean isAtSetpoint() {
        if (mRunning == Running.OPENLOOP) {
            return false;
        }
        return Math.abs(mTalonFX.getPosition().getValueAsDouble()
                - mRunningVelocitySetpoint) < IntakeConstants.velocityEqualityTolerance;
    }

    /* STATE ACCESSORS */

    public Running getState() {
        return mRunning;
    }

    public double getVelocity() {
        return mTalonFX.getRotorVelocity().getValueAsDouble();
    }

    /* STATE SETTING */

    public void setState(Running run) {
        mRunning = run;
    }

    public void setNeutralMode(NeutralModeValue mode) {
        mTalonFX.setNeutralMode(mode);
    }

    public void setOverride(double velocity) {
        mRunning = Running.OVERRIDE;
        mRunningVelocitySetpoint = velocity;
        mTalonFX.setControl(mRunningVelocityControl.withVelocity(velocity));
    }

    public void setOpenLoop(double output) {
        mRunning = Running.OPENLOOP;
        mRunningOpenLoopOutput = output;
        mTalonFX.setControl(mRunningOpenLoopControl.withOutput(output));
    }

    public double getSetpoint() {
        return mRunningVelocitySetpoint;
    }

    public double getVoltage() {
        return mTalonFX.getMotorVoltage().getValueAsDouble();
    }

    public ArrayList<TalonFX> getMotors() {
        var motors = new ArrayList<TalonFX>();
        motors.add(mTalonFX);
        return motors;
    }
}
