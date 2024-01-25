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
import frc.team6014.lib.math.Gearbox;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem mInstance;

    public TalonFX mRunningMotor;
    public TalonFX mAngleMotor;
    public Gearbox mGearbox = IntakeConstants.gearbox;

    public DutyCycleEncoder mBoreEncoder;
    public DigitalInput mBeamBreakSensor;

    private Position mPosition;
    private double mPositionSetpoint;
    private PositionVoltage mPositionControl;
    private double mAngleOpenLoopOutput;
    private DutyCycleOut mAngleOpenLoopControl;

    private Running mRunning;
    private double mRunningVelocitySetpoint;
    private double mRunningOpenLoopOutput;
    private DutyCycleOut mRunningOpenLoopControl;
    private VelocityVoltage mRunningVelocityControl;

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

        /** 
         * REMOVED GRAVITY-FF as Falcon's encoder is not directly connected
         * to the output shaft (TalonFX cannot calculate FF via kG * cos(Falcon encoder angle)).
         * If we want to add GRAVITY-FF, we'll have
         * to move the PID loop out of the Falcon and use a PIDController.
         */

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

    /* CALCULATIONS */
    public double drivenToDriver(double revolutions) {
        return revolutions * (1.0 / mGearbox.getRatio());
    }

    public double driverToDriven(double revolutions) {
        return mGearbox.calculate(revolutions);
    }

    /* ENCODERS */
    // theoretically, Falcon position / 72 = Bore encoder position + offset at all times

    public double getFalconPosition() {
        return mAngleMotor.getPosition().getValueAsDouble();
    }

    public double getBoreEncoderPosition() {
        return mBoreEncoder.getAbsolutePosition();
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
        return Math.abs(mAngleMotor.getPosition().getValueAsDouble() - mPositionSetpoint) < IntakeConstants.positionEqualityTolerance;
    }

    public boolean isAtVelocitySetpoint() {
        if (mRunning == Running.OPENLOOP) {
            return false;
        }
        return Math.abs(mRunningMotor.getPosition().getValueAsDouble() - mRunningVelocitySetpoint) < IntakeConstants.velocityEqualityTolerance;
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
                break;

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
}
