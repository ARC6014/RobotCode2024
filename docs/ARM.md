# [`Arm Subsystem`](/src/main/java/frc/robot/subsystems/ArmSubsystem.java)
CANBUS: CANivore
- `boreEncoder` as external encoder
- `MotionMagicVoltage` for voltage-driven Motion Magic

<br>

```java
public enum ArmControlState {
    /** open-loop control */
    OPEN_LOOP,

    /** intaking position */
    INTAKE,

    /** near speaker shooting position - tunable */
    SPEAKER_SHORT,

    /** far from speaker shooting position - NOT tunable */
    SPEAKER_LONG,

    /** amp shooting position */
    AMP,

    /** neutral - in brake */
    HOLD,

    /** zero position with respect to hard stop */
    ZERO,

    /** interpolation */
    POSE_T,

    /** look up table setup **/
    LOOKUP,

    /** FF characterization test */
    CHARACTERIZATION,

    /** CLIMBING CLOSED POSITION */
    CLIMB,

    /** intake from source zone when intake is broken */
    INTAKE_FROM_SOURCE,
  }
```
<br>


**State Setters:**
- `setArmPercentOutput()`
    - sets current state to `OPEN_LOOP`
    - sets `targetOutput` to given percent output
- `setArmAngleMotionMagic`
    - sets `setpoint` to given angle

**Interpolation:**
Look-up table for interpolation using LimeLight:
- `getAngleFromLookUp()` returns the calculated angle with respect to robot's current position to the speaker

**Checker:**
- `isBoreEncoderAlive()`
    - if bore is not connected, switch to Falcon encoder
<br>
