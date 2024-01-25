# `IntakeSubsystem`

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
    /** neutral/idle (coast) */
    NEUTRAL,
    /** custom setpoint/position */
    OVERRIDE,
    /** openloop control */
    OPENLOOP,
}
```

## Command

## TODO:
- [ ] PID constants (both angle & velocity)
- [ ] Bore encoder position offset
- [ ] \(Optional) Intake position reset if we ram it into the drivebase

