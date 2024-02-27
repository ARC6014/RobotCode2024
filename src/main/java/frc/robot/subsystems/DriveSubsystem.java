// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.team6014.lib.drivers.SwerveModuleBase;
import frc.team6014.lib.util.LoggedTunableNumber;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;
import io.github.oblarg.oblog.Loggable;

public class DriveSubsystem extends SubsystemBase implements Loggable {
  // Swerve numbering:
  // 0 1
  // 2 3

  private static DriveSubsystem mInstance;

  private final Trigger brakeModeTrigger;

  private final Command brakeModeCommand;

  public SwerveModuleBase[] mSwerveModules; // collection of modules
  private SwerveModuleState[] states; // collection of modules' states
  private ChassisSpeeds desiredChassisSpeeds; // speeds relative to the robot chassis

  public SwerveDriveOdometry mOdometry;

  private double[] velocityDesired = new double[4];
  private double[] velocityCurrent = new double[4];
  private double[] angleDesired = new double[4];

  WPI_Pigeon2 mGyro = new WPI_Pigeon2(Constants.Pigeon2CanID, Constants.CANIVORE_CANBUS);
  LimelightSubsystem mLL = LimelightSubsystem.getInstance();

  private boolean isLocked = false;

  private ProfiledPIDController snapPIDController = new ProfiledPIDController(DriveConstants.snapkP,
      DriveConstants.snapkI, DriveConstants.snapkD, DriveConstants.rotPIDconstraints);
  private static final LoggedTunableNumber<Number> kSnapP = new LoggedTunableNumber<Number>("Snap/kP",
      DriveConstants.snapkP);
  private static final LoggedTunableNumber<Number> kSnapI = new LoggedTunableNumber<Number>("Snap/kI",
      DriveConstants.snapkI);
  private static final LoggedTunableNumber<Number> kSnapD = new LoggedTunableNumber<Number>("Snap/kD",
      DriveConstants.snapkD);

  private final Timer snapTimer = new Timer();

  /* Snap PID calculations */
  private double snapAngle = 0.0;
  private double lastRotTime = 0.0;
  private double timeSinceDrive = 0.0;
  private double timeSinceRot = 0.0;
  private double lastDriveTime = 0.0;

  public SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    mSwerveModules = new SwerveModuleBase[] {
        new SwerveModuleBase(0, "FL", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleFrontLeft.driveMotorID, Constants.SwerveModuleFrontLeft.angleMotorID,
            Constants.SwerveModuleFrontLeft.cancoderID, Constants.SwerveModuleFrontLeft.angleOffset,
            Constants.SwerveModuleFrontLeft.modulekS, Constants.SwerveModuleFrontLeft.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(1, "FR", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleFrontRight.driveMotorID, Constants.SwerveModuleFrontRight.angleMotorID,
            Constants.SwerveModuleFrontRight.cancoderID, Constants.SwerveModuleFrontRight.angleOffset,
            Constants.SwerveModuleFrontRight.modulekS, Constants.SwerveModuleFrontRight.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(2, "RL", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleRearLeft.driveMotorID, Constants.SwerveModuleRearLeft.angleMotorID,
            Constants.SwerveModuleRearLeft.cancoderID, Constants.SwerveModuleRearLeft.angleOffset,
            Constants.SwerveModuleRearLeft.modulekS, Constants.SwerveModuleRearLeft.modulekV),
            DriveConstants.swerveConstants),

        new SwerveModuleBase(3, "RR", SwerveModuleConstants.generateModuleConstants(
            Constants.SwerveModuleRearRight.driveMotorID, Constants.SwerveModuleRearRight.angleMotorID,
            Constants.SwerveModuleRearRight.cancoderID, Constants.SwerveModuleRearRight.angleOffset,
            Constants.SwerveModuleRearRight.modulekS, Constants.SwerveModuleRearRight.modulekV),
            DriveConstants.swerveConstants)
    };

    snapTimer.reset();
    snapTimer.start();

    snapPIDController.enableContinuousInput(-Math.PI, Math.PI);

    zeroHeading();

    mOdometry = new SwerveDriveOdometry(Constants.kinematics, getRotation2d(), getModulePositions());

    AutoBuilder.configureHolonomic(
        this::getPoseMeters,
        this::resetOdometry,
        this::getChassisSpeed,
        this::setClosedLoopStates,
        Constants.holonomicPoseConfig, () -> {
          if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            return true;
          }
          return false;
        }, this);

    poseEstimator = new SwerveDrivePoseEstimator(
        Constants.kinematics,
        getRotation2d(),
        getModulePositions(),
        new Pose2d());

    brakeModeTrigger = new Trigger(RobotState::isEnabled);

    brakeModeCommand = new SequentialCommandGroup(
        new InstantCommand(() -> {
          for (SwerveModuleBase mod : mSwerveModules) {
            mod.setNeutralMode2Brake(true);
          }
        }),
        new WaitCommand(1.5),
        new InstantCommand(() -> {
          for (SwerveModuleBase mod : mSwerveModules) {
            mod.setNeutralMode2Brake(false);
          }
        }));

  }

  public static DriveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new DriveSubsystem();
    }
    return mInstance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();
    updatePoseEstimatorWithVisionBotPose();
    poseEstimator.update(
        getRotation2d(),
        getModulePositions());
    brakeModeTrigger.whileTrue(brakeModeCommand);

    snapPIDController.setPID(kSnapP.get().doubleValue(), kSnapI.get().doubleValue(), kSnapD.get().doubleValue());
    // SmartDashboard.putNumber("Swerve Voltage 0 ",
    // getDriveMotors().get(0).getMotorOutputVoltage());
    // SmartDashboard.putNumber("Swerve Voltage 1",
    // getDriveMotors().get(1).getMotorOutputVoltage());
    // SmartDashboard.putNumber("Swerve Voltage 2",
    // getDriveMotors().get(2).getMotorOutputVoltage());
    // SmartDashboard.putNumber("Swerve Voltage 3",
    // getDriveMotors().get(3).getMotorOutputVoltage());

    SmartDashboard.putNumber("Robot X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot Y", poseEstimator.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Robot tetha", poseEstimator.getEstimatedPosition().getRotation().getDegrees());

  }

  /*
   * Manual Swerve Drive Method
   */

  public Rotation2d getOdometryHeading() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    /* Automatically correcting the heading based on pid */
    // rot = calculateSnapValue(xSpeed, ySpeed, rot);

    // if robot is field centric, construct ChassisSpeeds from field relative speeds
    // if not, construct ChassisSpeeds from robot relative speeds
    ChassisSpeeds velocity = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,
            getDriverCentricRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    // discretize the ChassisSpeeds
    desiredChassisSpeeds = ChassisSpeeds.discretize(velocity, 0.02);

    // TODO: Implement if needed
    // Heading Angular Velocity Deadband, might make a configuration option later.
    // Originally made by Team 1466 Webb Robotics.
    // Modified by Team 7525 Pioneers and BoiledBurntBagel of 6036
    // if (true) {
    // if (Math.abs(velocity.omegaRadiansPerSecond) < HEADING_CORRECTION_DEADBAND
    // && (Math.abs(velocity.vxMetersPerSecond) > HEADING_CORRECTION_DEADBAND
    // || Math.abs(velocity.vyMetersPerSecond) > HEADING_CORRECTION_DEADBAND)) {
    // velocity.omegaRadiansPerSecond =
    // swerveController.headingCalculate(getOdometryHeading().getRadians(),
    // lastHeadingRadians);
    // } else {
    // lastHeadingRadians = getOdometryHeading().getRadians();
    // }
    // }

    states = Constants.kinematics.toSwerveModuleStates(desiredChassisSpeeds);

    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed);

    /*
     * Sets open loop states
     */
    for (int i = 0; i < 4; i++) {
      mSwerveModules[i].setDesiredState(states[i], true);
      velocityDesired[i] = states[i].speedMetersPerSecond;
      velocityCurrent[i] = mSwerveModules[i].getVelocityMPS();
      angleDesired[i] = states[i].angle.getDegrees();
    }

    // log();
  }

  // /*
  // * ------------ RESET § LOCK
  // */

  public void resetOdometry(Pose2d pose) {
    // mGyro.reset();
    // mGyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 :
    // 1).getDegrees());
    mOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    // mOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(mGyro.getYaw());
  }

  public void resetOdometryRelativeToAlliance(Pose2d pose) {
    // mGyro.reset();
    mGyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 : 1).getDegrees());
    var rotation = getRotation2d();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      rotation = rotation.minus(Rotation2d.fromDegrees(180));
    } else if (DriverStation.getAlliance().get() == Alliance.Red) {
      rotation = rotation.plus(Rotation2d.fromDegrees(180));
    }
    mOdometry.resetPosition(rotation, getModulePositions(), pose);
  }

  public void resetOdometry(Rotation2d angle) {
    Pose2d pose = new Pose2d(getPoseMeters().getTranslation(), angle);
    mGyro.reset();
    mGyro.setYaw(angle.getDegrees());
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  public void resetPoseEstimator(Pose2d pose) {
    poseEstimator.resetPosition(
        getRotation2d(),
        getModulePositions(),
        pose);

  }

  public void resetSnapPID() {
    snapPIDController.reset(getRotation2d().getRadians());
  }

  public void zeroHeading() {
    double before = mGyro.getRotation2d().getRadians();
    mGyro.reset();
    mGyro.setYaw(0);

    snapAngle = snapAngle - before;
  }

  public void stop() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
    }
  }

  public void resetToAbsolute() {
    for (SwerveModuleBase module : mSwerveModules) {
      module.stop();
      module.resetToAbsolute();
    }
  }

  public void lockSwerve(boolean should) {
    isLocked = should;
  }

  // /*
  // * ----------- SETTERS
  // */

  public void updateOdometry() {
    mOdometry.update(
        getRotation2d(),
        getModulePositions());
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  /*
   * Will be used in Auto by PPSwerveControllerCommand
   */

  public synchronized void setClosedLoopStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }
    mSwerveModules[0].setDesiredState(desiredStates[0], false);
    mSwerveModules[1].setDesiredState(desiredStates[1], false);
    mSwerveModules[2].setDesiredState(desiredStates[2], false);
    mSwerveModules[3].setDesiredState(desiredStates[3], false);

  }

  /*
   * @param speeds The desired ChassisSpeeds
   * 
   * Outputs commands to the robot's drive motors given robot-relative
   * ChassisSpeeds
   * 
   * Namely, driveRobotRelative or drive
   * 
   */
  public void setClosedLoopStates(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = Constants.kinematics.toSwerveModuleStates(speeds);
    setClosedLoopStates(desiredStates);
  }

  // /*
  // * ----------- GETTERS
  // */

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
        .times(DriveConstants.invertGyro ? -1 : 1);
  }

  // Returns gyro angle relative to alliance station
  public Rotation2d getDriverCentricRotation2d() {
    return DriverStation.getAlliance().get() == Alliance.Red
        ? Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle(), 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1)
        : Rotation2d.fromDegrees(Math.IEEEremainder(mGyro.getAngle() + 180, 360.0))
            .times(DriveConstants.invertGyro ? -1 : 1);
  }

  public Pose2d getPoseMeters() {
    return mOdometry.getPoseMeters();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        mSwerveModules[0].getPosition(),
        mSwerveModules[1].getPosition(),
        mSwerveModules[2].getPosition(),
        mSwerveModules[3].getPosition(),
    };
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModuleBase mod : mSwerveModules) {
      states[mod.getModuleNumber()] = mod.getState();
      mod.getDriveMotor();
    }
    return states;
  }

  /*
   * @return the desired wheel speeds in meters per second.
   *
   * Namely, getRobotRelativeSpeeds or getCurrentSpeeds
   */
  public ChassisSpeeds getChassisSpeed() {
    return Constants.kinematics.toChassisSpeeds(mSwerveModules[0].getState(), mSwerveModules[1].getState(),
        mSwerveModules[2].getState(), mSwerveModules[3].getState());
  }

  public ArrayList<WPI_TalonFX> getDriveMotors() {
    ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>();
    for (SwerveModuleBase module : mSwerveModules) {
      motors.add(module.getDriveMotor());
    }
    return motors;
  }

  // /*
  // * -------- OTHER
  // */

  // basically zero everything
  public void calibrate() {
    for (SwerveModuleBase mod : mSwerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    }
  }

  /*
   * @return true when a path should be flipped to the red side of the field
   */
  public boolean shouldAllianceFlipToRed() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    return alliance.get() == Alliance.Red;
  }

  private double calculateSnapValue(double xSpeed, double ySpeed, double rot) {

    double output = rot;

    if (Math.abs(rot) >= 0.05)
      lastRotTime = snapTimer.get();
    snapAngle = getRotation2d().getRadians();

    if (Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05)
      lastDriveTime = snapTimer.get();

    timeSinceRot = snapTimer.get() - lastRotTime;
    timeSinceDrive = snapTimer.get() - lastDriveTime;

    if (Math.abs(rot) < 0.05 && (Math.abs(xSpeed) >= 0.05 || Math.abs(ySpeed) >= 0.05)) { /* time since drive was 0.2 */
      output = snapPIDController.calculate(getRotation2d().getRadians(), snapAngle);
    }

    return output;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    System.out.println("VOLLT");
    for (var module : mSwerveModules) {
      module.setVoltage(volts);
    }

  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;

    for (var module : mSwerveModules) {
      driveVelocityAverage += module.getVelocityRPM() * 60;
    }

    return driveVelocityAverage / 4.0;
  }

  public void updatePoseEstimatorWithVisionBotPose() {
    if (mLL.getBotPose2d_field().getX() == 0.0) { // invalid LL data
      return;
    }

    // distance from current pose to vision estimated pose
    double poseDifference = poseEstimator.getEstimatedPosition().getTranslation()
        .getDistance(mLL.getBotPose2d_field().getTranslation());

    if (mLL.getID() != 0 || mLL.getID() != -1) { // is ID valid // needs to be updated, there are more conditions
      double xyStds;
      double degStds;

      if (mLL.getNumTargets() >= 2) { // trust for multiple tags
        xyStds = 0.5;
        degStds = 6;
      } else if (mLL.getArea() > 0.8 && poseDifference < 0.5) { // trust for 1 close target
        xyStds = 1.0;
        degStds = 12;
      } else if (mLL.getArea() > 0.1 && poseDifference < 0.3) { // trust for 1 far target
        xyStds = 2.0;
        degStds = 30;
      } else { // no trust
        return;
      }

      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
      poseEstimator.addVisionMeasurement(mLL.getBotPose2d_field(),
          Timer.getFPGATimestamp() - mLL.getLatency() / 1000.0);
    }
  }
}