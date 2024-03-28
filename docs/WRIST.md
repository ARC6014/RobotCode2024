# [`Wrist Subsystem`](/src/main/java/frc/robot/subsystems/WristSubsystem.java)
CANBUS: CANivore

- `boreEncoder` as external encoder
- `MotionMagicVoltage` for voltage-driven Motion Magic

<br>

```java 
public enum Position {
        /** ready for intaking */
        OPEN,
        /** closed to secure */
        CLOSED,
        /** custom setpoint/position */
        OVERRIDE,
        /** openloop control */
        OPENLOOP,
        /** for break mode in open loop (default) */
        HOLD,
    }
```

**Checker:**
- `isBoreEncoderAlive()`
    - if bore is not connected, switch to Falcon encoder
<br>

