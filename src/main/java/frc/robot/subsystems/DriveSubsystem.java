// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.team6014.lib.drivers.SwerveModuleBase;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.util.SwerveUtils.SwerveModuleConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;

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

  @Log.Gyro
  WPI_Pigeon2 mGyro = new WPI_Pigeon2(Constants.Pigeon2CanID, Constants.CANIVORE_CANBUS);

  private boolean isLocked = false;

  // private ProfiledPIDController snapPIDController = new
  // ProfiledPIDController(DriveConstants.snapkP,
  // DriveConstants.snapkI, DriveConstants.snapkD,
  // DriveConstants.rotPIDconstraints);

  private final Timer snapTimer = new Timer();

  public SwerveDrivePoseEstimator poseEstimator;

  // LOGS

  private DoubleArrayLogEntry velocityDesiredLog;
  private DoubleArrayLogEntry velocityActualLog;

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

    // snapPIDController.enableContinuousInput(-Math.PI, Math.PI); // ensure that
    // the PID controller knows -180 and 180 are

    zeroHeading();

    mOdometry = new SwerveDriveOdometry(Constants.kinematics, getRotation2d(), getModulePositions());

    for (SwerveModuleBase module : mSwerveModules) {
      RobotContainer.mOrchestra.addInstrument(module.getDriveMotor());
      RobotContainer.mOrchestra.addInstrument(module.getAngleMotor());
    }

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
    logInit();
    // new StartEndCommand(() -> {
    // for (SwerveModuleBase mod : mSwerveModules) {
    // mod.setNeutralMode2Brake(true);
    // }
    // }, () -> {
    // Timer.delay(1.5);
    // for (SwerveModuleBase mod : mSwerveModules) {
    // mod.setNeutralMode2Brake(false);
    // }
    // });
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
    poseEstimator.update(
        getRotation2d(),
        getModulePositions());
    brakeModeTrigger.whileTrue(brakeModeCommand);

    SmartDashboard.putNumber("Voltage 0 ", getDriveMotors().get(0).getMotorOutputVoltage());
    SmartDashboard.putNumber("Voltage 1", getDriveMotors().get(1).getMotorOutputVoltage());
    SmartDashboard.putNumber("Voltage 2", getDriveMotors().get(2).getMotorOutputVoltage());
    SmartDashboard.putNumber("Voltage 3", getDriveMotors().get(3).getMotorOutputVoltage());

    SmartDashboard.putNumber("Bus Voltage 0 ", getDriveMotors().get(0).getBusVoltage());
    SmartDashboard.putNumber("Bus Voltage 1", getDriveMotors().get(1).getBusVoltage());
    SmartDashboard.putNumber("Bus Voltage 2", getDriveMotors().get(2).getBusVoltage());
    SmartDashboard.putNumber("Bus Voltage 3", getDriveMotors().get(3).getBusVoltage());

    SmartDashboard.putNumber("Current 0", getDriveMotors().get(0).getOutputCurrent());
    SmartDashboard.putNumber("Current 1", getDriveMotors().get(1).getOutputCurrent());
    SmartDashboard.putNumber("Current 2", getDriveMotors().get(2).getOutputCurrent());
    SmartDashboard.putNumber("Current 3", getDriveMotors().get(3).getOutputCurrent());

    log();

  }

  public void logInit() {
    DataLog log = DataLogManager.getLog();
    velocityDesiredLog = new DoubleArrayLogEntry(log, "/DriveSubsystem/velocityDesired");
    velocityActualLog = new DoubleArrayLogEntry(log, "/DriveSubsystem/velocityActual");
  }

  public void log() {
    if (velocityDesired != null) {
      velocityDesiredLog.append(velocityDesired);
    }
    if (velocityCurrent != null) {
      velocityActualLog.append(velocityCurrent);
    }
  }

  /*
   * Manual Swerve Drive Method
   */

  public void swerveDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // if robot is field centric, construct ChassisSpeeds from field relative speeds
    // if not, construct ChassisSpeeds from robot relative speeds
    desiredChassisSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getDriverCentricRotation2d())
        : new ChassisSpeeds(xSpeed, ySpeed, rot);

    states = Constants.kinematics.toSwerveModuleStates(desiredChassisSpeeds);

    if (isLocked) {
      states = new SwerveModuleState[] {
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(45)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(315)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(135)),
          new SwerveModuleState(0.1, Rotation2d.fromDegrees(225))
      };
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxSpeed); // normalizes wheel speeds to absolute
                                                                                  // threshold

    /*
     * Sets open loop states
     */
    for (int i = 0; i < 4; i++) {
      mSwerveModules[i].setDesiredState(states[i], true);
      velocityDesired[i] = states[i].speedMetersPerSecond;
      velocityCurrent[i] = mSwerveModules[i].getVelocityMPS();
      angleDesired[i] = states[i].angle.getDegrees();
    }

    log();
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

  // basically zero everything
  public void calibrate() {
    for (SwerveModuleBase mod : mSwerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)), true);
    }
  }

  /*
   * Setters
   */
  public void resetOdometry(Pose2d pose) {
    // original code: swervePoseEstimator.resetPosition(getYaw(), getPositions(),
    // pose);
    // TODO: may also use the code below
    // swervePoseEstimator.resetPosition(getRotation2d(), getModulePositions(),
    // pose);
    mGyro.reset();
    mGyro.setYaw(pose.getRotation().times(DriveConstants.invertGyro ? -1 : 1).getDegrees());
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
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
    // snapPIDController.reset(getRotation2d().getRadians());
  }

  public void zeroHeading() {
    mGyro.reset();
    // mGyro.setYaw(0);
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

  public void updateOdometry() {
    mOdometry.update(
        getRotation2d(),
        getModulePositions());
  }

  public void setPose(Pose2d pose) {
    mOdometry.resetPosition(mGyro.getRotation2d(), getModulePositions(), pose);
  }

  /*
   * Getters
   */

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

  public ArrayList<WPI_TalonFX> getDriveMotors() {
    ArrayList<WPI_TalonFX> motors = new ArrayList<WPI_TalonFX>();
    for (SwerveModuleBase module : mSwerveModules) {
      motors.add(module.getDriveMotor());
    }
    return motors;
  }

  public Command orchestraCommand() {
    return startEnd(
        () -> {
          RobotContainer.mOrchestra.loadMusic("nevergonnagiveyouup.chrp");

          RobotContainer.mOrchestra.play();
        },
        () -> {
          RobotContainer.mOrchestra.stop();
        })
        .withName("Orchestra");
  }

  /*
   * public double getPitch() {
   * return mGyro.getPitch().getValue();
   * }
   * /*
   * public double getYaw() {
   * return mGyro.getYaw().getValue();
   * }
   */

}