package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class WristSubsystem extends SubsystemBase {
    private static WristSubsystem mInstance;

    private TalonFX mTalonFX;
    private Gearbox mGearbox = IntakeConstants.gearbox;
    public DutyCycleEncoder mBoreEncoder = new DutyCycleEncoder(IntakeConstants.boreEncoderDioId);

    private Position mPosition;
    private double mPositionSetpoint;
    private PositionVoltage mPositionControl;
    private double mAngleOpenLoopOutput;
    private DutyCycleOut mAngleOpenLoopControl;

    public WristSubsystem() {
        mTalonFX = new TalonFX(IntakeConstants.angleMotorId, Constants.CANIVORE_CANBUS);
        mPosition = Position.CLOSED;

        /* mAngleMotor + PositionControl setup */
        // TODO: fix PID constants
        var angleMotorConfigs = new Slot0Configs();
        angleMotorConfigs.kP = IntakeConstants.ANGLE_kP;
        angleMotorConfigs.kI = IntakeConstants.ANGLE_kI;
        angleMotorConfigs.kD = IntakeConstants.ANGLE_kD;

        mTalonFX.getConfigurator().apply(angleMotorConfigs);
        mTalonFX.setNeutralMode(NeutralModeValue.Brake);

        // TODO: mBoreEncoder.getAbsolutePosition() + IntakeConstants.positionOffset()
        // must output a position relative to the horizontal, which is 0
        resetToAbsolute();
        mPositionSetpoint = mTalonFX.getPosition().getValueAsDouble();
        mPositionControl = new PositionVoltage(mPositionSetpoint);

        mAngleOpenLoopOutput = 0;
        mAngleOpenLoopControl = new DutyCycleOut(0);
    }

    public static WristSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new WristSubsystem();
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

    @Override
    public void periodic() {
        switch (mPosition) {
            case OPENLOOP:
                mTalonFX.setControl(mAngleOpenLoopControl);
                break;

            default:
                /* GRAVITY FF */
                mTalonFX.setControl(mPositionControl);
                break;
        }

        // // STOP ANGLE MOTOR IF WE ARE GOING INTO THE DRIVEBASE
        // if (getBoreEncoderPosition() < IntakeConstants.stopPosition) {
        //     setOpenLoop(0);
        //     mTalonFX.stopMotor();
        //     resetToAbsolute();
        // }

        SmartDashboard.putNumber("Bore Angle", getBoreEncoderPosition());
    }

    /* CALCULATIONS */
    public double drivenToDriver(double revolutions) {
        return revolutions * (1.0 / mGearbox.getRatio());
    }

    public double driverToDriven(double revolutions) {
        return mGearbox.calculate(revolutions);
    }
    
    /* ENCODERS */
    // theoretically, Falcon position / 72 = Bore encoder position at all
    // times

    public double getFalconPosition() {
        return mTalonFX.getPosition().getValueAsDouble();
    }

    /** Bore encoder reading (without position offset) */
    public double getBoreEncoderPosition() {
        return mBoreEncoder.getAbsolutePosition();
    }

    public void resetToAbsolute() {
        double position = drivenToDriver(mBoreEncoder.getAbsolutePosition());
        mTalonFX.setPosition(position);
    }

    public boolean isAtSetpoint() {
        if (mPosition == Position.OPENLOOP) {
            return false;
        }
        return Math.abs(mTalonFX.getPosition().getValueAsDouble()
                - mPositionSetpoint) < IntakeConstants.positionEqualityTolerance;
    }

    public Position getState() {
        return mPosition;
    }

    public void setState(Position pos) {
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
                return;
        }

        mPositionControl.Position = mPositionSetpoint;
        mTalonFX.setControl(mPositionControl);
    }

    public void setOverride(double position) {
        mPosition = Position.OVERRIDE;
        mPositionSetpoint = position;
        mPositionControl.Position = mPositionSetpoint;
        mTalonFX.setControl(mPositionControl);
    }

    public void setOpenLoop(double output) {
        mPosition = Position.OPENLOOP;
        mAngleOpenLoopOutput = output;
        mTalonFX.setControl(mAngleOpenLoopControl.withOutput(output));
    }

    public ArrayList<TalonFX> getMotors() {
        var motors = new ArrayList<TalonFX>();
        motors.add(mTalonFX);
        return motors;
    }

}
