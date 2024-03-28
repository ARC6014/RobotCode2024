# [`Intake Subsystem`](/src/main/java/frc/robot/subsystems/IntakeSubsystem.java)
CANBUS: rio

## Class Members
```java
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
private PositionVoltage mPositionControl; // PID
private double mAngleOpenLoopOutput;
private DutyCycleOut mAngleOpenLoopControl;

/* RUNNING/INTAKING MOTOR CONTROL */
private Running mRunning;
private double mRunningVelocitySetpoint;
private VelocityVoltage mRunningVelocityControl; // PID
private double mRunningOpenLoopOutput;
private DutyCycleOut mRunningOpenLoopControl;
```

## States
```java

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
```


