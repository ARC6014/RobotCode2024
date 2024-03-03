package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.IntakeConstants;
import frc.team6014.lib.math.Conversions;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    /* MOTORS */
    public TalonFX mTalonFX;

    /* SENSORS */
    public DigitalInput mBeamBreakSensor;

    /* RUNNING/INTAKING MOTOR CONTROL */
    private Running mRunning;
    /** unit: rps */
    private double mRunningVelocitySetpoint;
    private VelocityVoltage mRunningVelocityControl;

    private double mRunningOpenLoopOutput;
    private DutyCycleOut mRunningOpenLoopControl;

    private TalonFXConfiguration motorConfig;

    private NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

    public IntakeSubsystem() {
        mTalonFX = new TalonFX(IntakeConstants.RUNNING_MOTOR_ID, Constants.RIO_CANBUS);

        mBeamBreakSensor = new DigitalInput(IntakeConstants.BEAM_BREAK_ID);

        mRunning = Running.S_DOWN;

        motorConfig = new TalonFXConfiguration();
        motorConfig.Voltage.PeakForwardVoltage = 12;
        motorConfig.Voltage.PeakForwardVoltage = -12;
        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.5;
        motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;
        motorConfig.Slot0.kP = IntakeConstants.RUN_kP;
        motorConfig.Slot0.kI = IntakeConstants.RUN_kI;
        motorConfig.Slot0.kD = IntakeConstants.RUN_kD;
        motorConfig.Slot0.kS = IntakeConstants.RUN_kS;
        motorConfig.Slot0.kV = IntakeConstants.RUN_kV;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        mTalonFX.getConfigurator().apply(motorConfig);

        mRunningVelocitySetpoint = 0;
        mRunningVelocityControl = new VelocityVoltage(mRunningVelocitySetpoint);

        mRunningOpenLoopOutput = 0;
        mRunningOpenLoopControl = new DutyCycleOut(0);
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
        /** neutral */
        NEUTRAL,
        /** stop motor */
        S_DOWN,
        /** custom setpoint/position */
        OVERRIDE,
        /** openloop control */
        OPENLOOP,
        /** testing when feeder is not assembled */
        FEEDING_SHOOTER,
    }

    @Override
    public void periodic() {
        // if ((!mBeamBreakSensor.get()) && (mRunning != Running.REVERSE)) {
        // setState(Running.S_DOWN);
        // mTalonFX.stopMotor();
        // return;
        // }

        switch (mRunning) {
            case FORWARD:
                mRunningVelocitySetpoint = IntakeConstants.FORWARD_VELOCITY;
                break;

            case REVERSE:
                mRunningVelocitySetpoint = IntakeConstants.REVERSE_VELOCITY;
                break;

            case NEUTRAL:
                mRunningVelocitySetpoint = 0;
                break;

            case S_DOWN:
                mTalonFX.stopMotor();
                break;

            case OVERRIDE:
                break;

            case FEEDING_SHOOTER:
                mRunningVelocitySetpoint = IntakeConstants.FEEDER_VELOCITY;
                break;

            default:
                mRunningVelocitySetpoint = 0;
                break;
        }

        if (mRunning == Running.OPENLOOP || mRunning == Running.S_DOWN)
            mTalonFX.setControl(mRunningOpenLoopControl);
        else {
            mRunningVelocityControl.Velocity = mRunningVelocitySetpoint;
            mTalonFX.setControl(mRunningVelocityControl);

        }

        SmartDashboard.putString("Idle Mode", this.kNeutralMode.toString());
    }

    public boolean getBeambreak() {
        return mBeamBreakSensor.get();
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

    /** toggles neutral mode of motor */
    public void setNeutralMode() {
        this.kNeutralMode = (kNeutralMode == NeutralModeValue.Brake) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        mTalonFX.setNeutralMode(this.kNeutralMode);
    }

    public void setOverride(double velocity) {
        mRunning = Running.OVERRIDE;
        mRunningVelocitySetpoint = velocity;
        mTalonFX.setControl(mRunningVelocityControl.withVelocity(velocity));
    }

    public void setOpenLoop(double output) {
        output = Conversions.getSmartVoltage(output, RobotContainer.mPDH.getVoltage());
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
