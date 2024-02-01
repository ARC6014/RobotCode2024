package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    /* MOTORS */
    public TalonFX mTalonFX;

    /* SENSORS */
    //public DigitalInput mBeamBreakSensor;

    /* RUNNING/INTAKING MOTOR CONTROL */
    private Running mRunning;
    private double mRunningVelocitySetpoint;
    private VelocityVoltage mRunningVelocityControl;
    private double mRunningOpenLoopOutput;
    private DutyCycleOut mRunningOpenLoopControl;

    public IntakeSubsystem() {
        mTalonFX = new TalonFX(IntakeConstants.runningMotorId, Constants.RIO_CANBUS);

        //mBeamBreakSensor = new DigitalInput(IntakeConstants.beamBreakSensorDioId);

        mRunning = Running.NEUTRAL;

        /* mTalonFX + VelocityControl setup */
        // TODO: fix PID constants
        var runningMotorConfigs = new Slot0Configs();
        runningMotorConfigs.kP = IntakeConstants.RUN_kP;
        runningMotorConfigs.kI = IntakeConstants.RUN_kI;
        runningMotorConfigs.kD = IntakeConstants.RUN_kD;

        mTalonFX.getConfigurator().apply(runningMotorConfigs);

        mRunningVelocitySetpoint = 0;
        mRunningVelocityControl = new VelocityVoltage(mRunningVelocitySetpoint);
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
    }

    @Override
    public void periodic() {
        // if (mBeamBreakSensor.get()) {
        //     setState(Running.NEUTRAL);
        // }

        switch (mRunning) {
            case OPENLOOP:
                mTalonFX.setControl(mRunningOpenLoopControl);
                break;

            case NEUTRAL:
                mTalonFX.stopMotor();

            default:
                mTalonFX.setControl(mRunningVelocityControl);
                break;
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

    /* STATE SETTING */

    public void setState(Running run) {
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
                mTalonFX.stopMotor();
                return;

            case OVERRIDE:
                break;

            case OPENLOOP:
                // don't set control if it's open loop, just return
                return;
        }

        mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
        mTalonFX.setControl(mRunningVelocityControl);
    }

    public void setOverride(double velocity) {
        mRunning = Running.OVERRIDE;
        mRunningVelocitySetpoint = velocity;
        mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
        mTalonFX.setControl(mRunningVelocityControl);
    }

    public void setOpenLoop(double output) {
        mRunning = Running.OPENLOOP;
        mRunningOpenLoopOutput = output;
        mRunningOpenLoopControl.Output = mRunningOpenLoopOutput;
        mTalonFX.setControl(mRunningOpenLoopControl);
    }

    public ArrayList<TalonFX> getMotors() {
        var motors = new ArrayList<TalonFX>();
        motors.add(mTalonFX);
        return motors;
    }
}
