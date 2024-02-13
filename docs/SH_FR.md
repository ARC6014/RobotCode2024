# [`Shooter & Feeder`](/src/main/java/frc/robot/subsystems/ShooterSubsystem.java)

## Class Members
```java
//Hardware
private CANSparkMax m_master;
private CANSparkMax m_slave;
private CANSparkMax m_feeder;
private DigitalInput m_beamBreaker;

//Constants
private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

//State and Instances
private static ShooterSubsystem m_instance;
private ShooterState m_shootState;
private FeederState m_feederState;

//PID Controller
private SparkPIDController m_masterPIDController;
private SparkPIDController m_slavePIDController;
```

## States
```java
// Position of the shooter
public enum ShooterState {
    OPEN_LOOP,
    CLOSED,
    AMP,
    SPEAKER,
}

//Self Explanatory Enough
public enum FeederState {
    LET_HIM_COOK,
    STOP_WAIT_A_SEC,
}
```

## [Commands](/src/main/java/frc/robot/commands/shooter)

```java
public class SFeederCommand extends Command {
    //full open loop with default option of 0.5 Out
}

public class ShooterCommand extends Command {
    withOpenLoop(double output);
    withShooterState(ShooterState level);
}
```

