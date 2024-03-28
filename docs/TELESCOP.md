# [`Telescopic Subsystem`](/src/main/java/frc/robot/subsystems/TelescopicSubsystem.java)
CANBUS: rio

```java
public enum TelescopicState {
    /* neutral - in brake */
    HOLD,
    /* open loop control */
    OPEN_LOOP,
    /* closed-loop motion magic */
    CLIMB,
    /* stop motors */
    STOP,
  }
```

2 separate states for left and right telescopics:
```java 
switch (leftState) {
      case HOLD:
        m_left_motor.setControl(new NeutralOut());
        break;
      case OPEN_LOOP:
        setLeftOutput();
        break;
      default:
        stopLeft();
        break;
    }

switch (rightState) {
      case HOLD:
        m_right_motor.setControl(new NeutralOut());
        break;
      case OPEN_LOOP:
        setRightOutput();
        break;
      default:
        stopRight();
        break;
    } 
```