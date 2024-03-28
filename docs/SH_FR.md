# [`Shooter Subsystem`](/src/main/java/frc/robot/subsystems/ShooterSubsystem.java) [`Feeder Subsystem `](/src/main/java/frc/robot/subsystems/FeederSubsystem.java)
CANBUS: rio

Shooter: 2 NEOs controlled by SparkMAX
Feeder: 1 775pro controlled by TalonSRX

## Class Members
```java
// Hardware
private CANSparkMax m_master;
private CANSparkMax m_slave;
private CANSparkMax m_feeder;
private DigitalInput m_beamBreaker;

// Constants
private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

// Interpolation
InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> map = new InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>();

// State and Instances
private static ShooterSubsystem m_instance;
private ShooterState m_shootState;
private FeederState m_feederState;

// PID Controller - not used
private SparkPIDController m_masterPIDController;
private SparkPIDController m_slavePIDController;
```

## States
```java
public enum ShooterState {
    /* open-loop */
    OPEN_LOOP,

    /* stopped */
    CLOSED,

    /* amp voltage */
    AMP,

    /* speaker long voltage (not tunable) */
    SPEAKER_LONG,

    /* speaker short a.k.a tunable voltage */
    SPEAKER_SHORT,

    /* interpolated voltage from lookup */
    LOOKUP,

    /* source intake */
    INTAKE_FROM_SOURCE,
  }

  public enum FeederState {
    /* feed to shooter */
    LET_HIM_COOK,

    /* eject */
    UPSI,

    /* stop feeder */
    STOP_WAIT_A_SEC,

    OPEN,

    /* feeder from intake */
    INTAKECEPTION,
  }
```



