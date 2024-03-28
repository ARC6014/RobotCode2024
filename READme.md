## Official 2024 Robot Code for Crescendo
[![CI](https://github.com/ARC6014/RobotCode2024/actions/workflows/main.yml/badge.svg)](https://github.com/ARC6014/RobotCode2024/actions/workflows/main.yml)

## Introducing ARC's 2024 Robot: Carabot
<img title="Carabot" alt="Carabot" src="/images/carabot.jpeg">

## Docs
Explanation of codes can be found under [docs](/docs):

- [Arm & Telescopic](/docs/ARM.md)
  - Cr: CAN
- [Intake](/docs/INTAKE.md)
  - Cr: Ouz
- [Shooter & Feeder](/docs/SH_FR.md)
  - Cr: Alia & Cr

## Autos
- 4PAllClosePieces (most used)
  - Shoot inside + 3 close notes with LL interpolation
  - approx. %85 precision of shooting 3/4 notes
- ShootInside
  - Only shoot and wait
- 3PFarRight+Far2nd (not optimized)
  - Shoot inside + get 2 notes from the centerline
- Disrupter Path
  - Disrupt the 4 notes in centerline


### Editing Guidelines:
- Every subsystem should have Open & Closed Loop commands.
- Open Loop should be bound to necessary Operator/Driver joysticks.
- Subsystem-specific constants should be capsulated in respective subsystem constant classes.
- Naming Conventions
  - Fields in form of `mName`
  - A DriveSubsystem instance in form of `mDrive` 
- Every subsystem should have a `getInstance()` method which would be called to get an instance of the subsystem. Subsystems shouldn’t be instantiated in `RobotContainer.java` as `Subsystem sub = new Subsystem()`!
- Every subsystem should include necessary debugging printed to SmartDashboard/Shuffleboard, such as encoder readings, zeroing booleans, etc.
- `Robot.java` should be avoided most of the time unless writing in `autoInit()` `teleopInit()` etc, `RobotContainer.java` suffices.

### Formatting code:
- Explanatory comments labeled `FIX!` and `TODO:` are a must.
- Every commit should include necessary changes done.