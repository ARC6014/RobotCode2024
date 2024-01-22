## Official 2024 Robot Code for Crescendo
### Editing Guidelines:
- Every subsystem should have Open & Closed Loop commands. Open Loop should be bound to necessary Operator/Driver joysticks.
- The Open Loop commands will be created under the Deneme commands section with respective titles.
- Subsystem-specific constants should be capsulated in respective subsystem constant classes.
- Every member of a subsystem should have their name in the convention `mName`. A DriveSubsystem instance would be `mDrive`, respectively. 
- Every subsystem should have a `getInstance()` method which would be called to get an instance of the subsystem. Subsystems shouldn’t be instantiated in `RobotContainer.java` as `Subsystem sub = new Subsystem()`!
- Every subsystem should include necessary debugging printed to SmartDashboard/Shuffleboard, such as encoder readings, zeroing booleans, etc.
- `Robot.java` should be avoided most of the time unless writing in `autoInit()` `teleopInit()` etc, `RobotContainer.java` suffices.

Formatting code:
- Explanatory comments labeled `FIX!` and `TODO:` are a must.
- Every commit should include necessary changes done.
