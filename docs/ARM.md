### `ArmSubsystem.java`
- `boreEncoder` as external encoder
- `MotionMagicTorqueCurrentFOC` for FOC and Motion Magic
- `PositionTorqueCurrentFOC` for keeping arm in position after reached setpoint

<br>

`ArmControlState`:
default state: `OPEN_LOOP`
- `OPEN_LOOP` for joystick-fed value
    - `setMotorOutput()` sets motor percent output

- `TORQUE_CONTROL` for holding position
    - `holdPosition()` holds the current mechanism angle

- `MOTION_MAGIC` for reaching setpoint using MM
    - `setArmAngleMotionMagic()` sets arm angle with MM
    - Note that MM uses Falcon internal encoder for now!

- default: stops (percent output=0)

<br>

- `isAtSetpointFalcon` and `isAtZeroFalcon` are currently using only Falcon encoders, implementation with Bore is yet to come.

**State Setters:**
- `setArmPosition()`
    - sets current state to `MOTION_MAGIC` 
    - sets the `setpoint` to given value
- `holdArmPosition()`
    - sets current state to `TORQUE_CONTROL`
- `setArmPercentOutput()`
    - sets current state to `OPEN_LOOP`
    - sets `targetOutput` to given percent output

<br>

**TODOS:**
- [ ] Configure `kP`, `kI`, `kD` for slots 0&1
- [ ] Configure necessary limits
- [ ] Check motor inverts
- [ ] Configure motor IDs & Bore port
- [ ] Configure angle tolerance&reset angle
- [ ] Configure arm setpoint angles
- [ ] Set Bore's `distancePerRotation`
- [ ] Check encoder tick to rotation conversions in all methods!

<br>

## `ArmOpenLoop.java`
- takes in joystick & one button
- joystick to move arm up&down, button to test one given setpoint

**TODOS:**
- [ ] Configure `targetAngle` for the button

<br>

## `ArmClosedLoop.java`
- closed loop command for arm which takes in a `setpoint` and `finishPoint`.
- will finish command when current angle is within a tolerance of the `finishPoint`
