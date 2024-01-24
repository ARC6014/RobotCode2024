package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    public TalonFX mRunningMotor;
    public TalonFX mAngleMotor;

    public DutyCycleEncoder mBoreEncoder;
    public DigitalInput mBeamBreakSensor;

    public Position mPosition;
    public double mPositionSetpoint;
    public PositionVoltage mPositionControl;
    public double mAngleOpenLoopOutput;
    public DutyCycleOut mAngleOpenLoopControl;

    public Running mRunning;
    public double mRunningVelocitySetpoint;
    public double mRunningOpenLoopOutput;
    public DutyCycleOut mRunningOpenLoopControl;
    public VelocityVoltage mRunningVelocityControl;

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
        angleMotorConfigs.kP = 24; // An error of 0.5 rotations results in 12 V output
        angleMotorConfigs.kI = 0; // no output for integrated error
        angleMotorConfigs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        // Gravity FF:
        // the sensor offset must be configured such that a position of 0 represents the arm being held horizontally forward.
        angleMotorConfigs.kG = 0; // TODO: measure voltage to hold intake horizontal
        angleMotorConfigs.GravityType = GravityTypeValue.Arm_Cosine;
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/closed-loop-requests.html#gravity-feedforward

        mAngleMotor.getConfigurator().apply(angleMotorConfigs);
        mAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        // TODO: mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset() 
        // must output a position relative to the horizontal, which is 0
        mAngleMotor.setPosition(mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset);
        mPositionSetpoint = mAngleMotor.getPosition().getValueAsDouble();
        mPositionControl = new PositionVoltage(mPositionSetpoint);

        /* mRunningMotor + VelocityControl setup */
        // TODO: fix PID constants
        var runningMotorConfigs = new Slot0Configs();
        runningMotorConfigs.kP = 24; // An error of 0.5 rotations results in 12 V output
        runningMotorConfigs.kI = 0; // no output for integrated error
        runningMotorConfigs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

        mRunningMotor.getConfigurator().apply(runningMotorConfigs);
        mAngleMotor.setNeutralMode(NeutralModeValue.Coast);

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
        OVERRIDE,
        OPENLOOP,
    }

    public enum Running {
        FORWARD,
        REVERSE,
        NEUTRAL,
        OVERRIDE,
        OPENLOOP,
    }

    @Override
    public void periodic() {
        // TODO: Control requests are automatically transmitted at a fixed update frequency.
        // does this mean that we do not have to setControl every periodic run?
        // https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/control-requests.html#changing-update-frequency
        // nevertheless I added them here just to be sure

        switch (mPosition) {
            case OPENLOOP:
                mAngleMotor.setControl(mAngleOpenLoopControl);
                break;
        
            default:
                mAngleMotor.setControl(mPositionControl);
                break;
        }

        switch (mRunning) {
            case OPENLOOP:
                mRunningMotor.setControl(mRunningOpenLoopControl);
                break;
        
            default:
                mRunningMotor.setControl(mRunningVelocityControl);
                break;
        }
        
        // VOLTAGE CHECK TO STOP MOTOR IF WE RAM THE INTAKE INTO THE CHASSIS
        // TODO: this is just a fix to stop the motor, does not readjust encoders etc.
        // we'll have to implement more code to make it usable after stopping
        if (mAngleMotor.getMotorVoltage().getValueAsDouble() > IntakeConstants.maxVoltageCutoff) {
            setAngleOpenLoop(0);
            mAngleMotor.stopMotor();
        }
    }

    public boolean isAtPositionSetpoint() {
        return Math.abs(mAngleMotor.getPosition().getValueAsDouble() - mPositionSetpoint) < IntakeConstants.positionEqualityTolerance;
    }

    public boolean isAtVelocitySetpoint() {
        return Math.abs(mRunningMotor.getPosition().getValueAsDouble() - mRunningVelocitySetpoint) < IntakeConstants.velocityEqualityTolerance;
    }

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
                break;

            case OVERRIDE:
                break;

            case OPENLOOP:
                break;
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
                mPositionSetpoint = IntakeConstants.openPosition;
                break;
        
            case CLOSED:
                mPositionSetpoint = IntakeConstants.closedPosition;
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
}
