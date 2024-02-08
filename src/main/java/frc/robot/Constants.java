// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.SwerveUtils.SwerveDriveConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants implements Loggable {
    public static final String CANIVORE_CANBUS = "CANivore";
    public static final String RIO_CANBUS = "rio";
    @Log
    public static int Pigeon2CanID = 60;

    public static final double wheelBaseLength = 0.56665;
    private static final double wheelBaseWidth = 0.56665;

    public static final double drivebaseRadius = Math.hypot(wheelBaseWidth / 2.0, wheelBaseLength / 2.0);
    public static final double maxModuleSpeed = 4.0;

    // TODO: Tune PID
    public static final HolonomicPathFollowerConfig holonomicPoseConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(1.0, 0, 0),
            new PIDConstants(5, 0, 0),
            maxModuleSpeed,
            drivebaseRadius,
            new ReplanningConfig(),
            0.02);

    // Module coordinates according to the chassis
    public static final Translation2d swerveModuleLocations[] = {
            new Translation2d(wheelBaseLength / 2, wheelBaseWidth / 2), // FL
            new Translation2d(wheelBaseLength / 2, -wheelBaseWidth / 2), // FR
            new Translation2d(-wheelBaseLength / 2, wheelBaseWidth / 2), // RL
            new Translation2d(-wheelBaseLength / 2, -wheelBaseWidth / 2) // RR
    };
    public static final Translation2d FRONTLEFTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2,
            wheelBaseWidth / 2);
    public static final Translation2d FRONTRIGHTMODULE_TRANSLATION2D = new Translation2d(wheelBaseLength / 2,
            -wheelBaseWidth / 2);
    public static final Translation2d REARLEFTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2,
            wheelBaseWidth / 2);
    public static final Translation2d REARRIGHTMODULE_TRANSLATION2D = new Translation2d(-wheelBaseLength / 2,
            -wheelBaseWidth / 2);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            FRONTLEFTMODULE_TRANSLATION2D, // FL
            FRONTRIGHTMODULE_TRANSLATION2D, // FR
            REARLEFTMODULE_TRANSLATION2D, // RL
            REARRIGHTMODULE_TRANSLATION2D); // RR
    public static final boolean IS_TATMIN = true;

    public static final class DriveConstants {
        public static final boolean isFieldOriented = true;
        public static final boolean invertGyro = true; // !! CCW+

        public static final int angleContinuousCurrentLimit = 20;
        public static final int anglePeakCurrentLimit = 35;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.2;
        public static final boolean driveEnableCurrentLimit = true;

        public static final NeutralMode angleMotorNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveMotorNeutralMode = NeutralMode.Brake;

        public static final double openLoopRamp = 0;
        public static final double closedLoopRamp = 0;

        public static final double drivePowerScalar = 0.55;
        public static final double driveSlewRateLimitX = 7;
        public static final double driveSlewRateLimitY = 7;
        public static final double driveSlewRateLimitRot = 12;

        public static final double angleGearboxRatio = 22.93;
        public static final double driveGearboxRatio = 6.59340659;
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        // PID and Feedforward
        @Config
        public static final double drivekP = 0.30;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.23; // prev: 0.016
        public static final double drivekV = 0.19;
        public static final double drivekA = 0;

        public static final double anglekP = 0.27;
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        // snap PID is not used currently
        public static final double snapkP = 2.5;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.01;

        /** will be used for FieldOrientedTurn */
        public static final double kRotControllerP = 0.15; // prev: 0.13
        public static final double kRotControllerD = 0.28;
        public static final double kRotControllerMaxVel = 5;
        public static final double kRotControllerTolerance = 3.5;

        public static final double maxSpeed = 5;

        public static final double maxTransSpeedMetersPerSecond = 3.3; // translation speed (x/y)
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI; // angular speed (omega)
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2); // angular
                                                                                                       // acceleration

        public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
                maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

        // added these two for LLalignment not sure if it makes sense
        private static final double maxTransAccelMetersPerSecSq = 2;
        public static final TrapezoidProfile.Constraints transPIDconstraints = new TrapezoidProfile.Constraints(
                maxTransSpeedMetersPerSecond, maxTransAccelMetersPerSecSq);

        public static SwerveDriveConstants swerveConstants = SwerveDriveConstants.generateSwerveConstants(
                angleContinuousCurrentLimit,
                anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit, driveContinuousCurrentLimit,
                drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp, closedLoopRamp);
    }

    // TODO: Configure each module's angle offset in calibration
    public static final class SwerveModuleFrontLeft {
        public static final int angleMotorID = 50;
        public static final int driveMotorID = 51;
        public static final int cancoderID = 01;
        public static final double angleOffset = -84.37 + 0.2;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleFrontRight {
        public static final int angleMotorID = 56;
        public static final int driveMotorID = 57;
        public static final int cancoderID = 02;
        public static final double angleOffset = 14.5;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearLeft {
        public static final int angleMotorID = 54;
        public static final int driveMotorID = 55;
        public static final int cancoderID = 03;
        public static final double angleOffset = -136.93 - 2;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearRight {
        public static final int angleMotorID = 52;
        public static final int driveMotorID = 53;
        public static final int cancoderID = 04;
        public static final double angleOffset = -73.47 - 0.26;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class ArmConstants {

        public static final int motorID = 20;
        public static final int boreChannel = 1;

        public static final Gearbox gearRatio = new Gearbox(1, 119.5);
        public static final boolean motorInverted = false;

        public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

        /** unit: rev/s */
        public static final double armCruiseVelocity = 150;
        /** unit: rev/s^2 */
        public static final double armAcceleration = 120;

        public static final double kP = 1.2;
        public static final double kD = 0.0;
        public static final double kI = 0;
        public static final double kG = 0; // TODO: Config
        public static final double kS = 0.14;
        public static final double kV = 0; // TODO: Config

        /** unit: rotations */
        public static final double positionOffset = Conversions.degreesToRevolutions(13.34); // TODO: Config starting
                                                                                             // position offset of bore

        /** unit: rotations */
        public static final double angleTolerance = Conversions.degreesToRevolutions(1);

        /* Arm angles for setpoints (relative to the zero of Bore) */
        /** unit: degrees */
        public static final double ZERO = 0;
        /** unit: degrees */
        public static final double INTAKE = 9 + 15;
        /** unit: degrees */
        public static final double SPEAKER_LONG = 30 + 15;
        /** unit: degrees */
        public static final double SPEAKER_SHORT = 11 + 15;
        /** unit: degrees */
        public static final double AMP = 90 + 15;
    }

    public static final class IntakeConstants {

        public static final int runningMotorId = 10;
        public static final int beamBreakSensorDioId = 2;

        /** unit: rps */
        public static final double forwardVelocity = 0;
        /** unit: rps */
        public static final double reverseVelocity = 0;

        // TODO: tolerance
        public static final double velocityEqualityTolerance = 0;

        /** unit: V/(rev/s) */
        public static final double RUN_kP = 0;
        /** unit: V/((rev/s) * s) */
        public static final double RUN_kI = 0;
        /** unit: Vs/(rev/s) */
        public static final double RUN_kD = 0;

    }

    public static final class WristConstants {
        public static final int angleMotorId = 11;
        public static final int boreEncoderDioId = 3;

        public static final Gearbox gearbox = new Gearbox(72, 1);

        /** REV Bore Encoder position offset, unit: revolutions */
        public static final double positionOffset = 0;
        /** REV Bore Encoder position, with the horizontal as 0, unit: revolutions */
        public static final double openPosition = 0;
        /** REV Bore Encoder position, with the horizontal as 0, unit: revolutions */
        public static final double closedPosition = 0;
        /** unit: revolutions */
        public static final double stopPosition = 0;

        /** unit: revolutions */
        public static final double positionEqualityTolerance = Conversions.degreesToRevolutions(0.5);

        /** unit: V/rev */
        public static final double ANGLE_kP = 0;
        /** unit: V/(rev * s) */
        public static final double ANGLE_kI = 0;
        /** unit: Vs/rev */
        public static final double ANGLE_kD = 0;

        /** unit: rev/s */
        public static final double armCruiseVelocity = 120;
        /** unit: rev/s^2 */
        public static final double armAcceleration = 70;

        // TODO: voltage cutoff
        public static final double maxVoltageCutoff = 0;
        // TODO: kG
        public static final double kG = 0;

    }

    public static final class ShooterConstants {

        /* Motors & Sensors */
        public static final int MASTER_MOTOR_ID = 30;
        public static final int SLAVE_MOTOR_ID = 31;
        public static final int FEEDER_MOTOR_ID = 32;
        public static final int BEAM_BREAK_ID = 4; // TODO: Config

        public static final double kMinOutput = -1; // TODO: Config
        public static final double kMaxOutput = 1; // TODO: Config

        /* PID & FF */
        public static final double kP = 0.000594; // TODO: Config
        public static final double kFF = 0.0002219; // TODO: Config .000085
        public static final double kIz = 0; // TODO: Config
        public static final double kD = 0.182; // TODO: Config burayı biraz daha artttırcaz 0.202 fazla geldi
        public static final double kI = 0; // TODO: Config

        /* Neutral Modes */
        public static final IdleMode FEEDER_MODE = IdleMode.kBrake;
        public static final IdleMode MASTER_MODE = IdleMode.kBrake;

        /* Inverts */
        // TODO: inverts
        public static final boolean masterInverted = true;
        public static final boolean slaveInverted = false;
        public static final boolean feederInverted = false;

        /* RPMs */
        public static final double maxRPM = 5700; // TODO: Config
        public static final double AMP_SHOOT_RPM = 0; // TODO: Config
        public static final double SPEAKER_SHOOT_RPM = 0; // TODO: Config
    }

    public static final class TelescopicConstants {
        public static final int MASTER_MOTOR_ID = 40; // TODO: Config
        public static final int SLAVE_MOTOR_ID = 41; // TODO: Config

        public static final int TELESCOPIC_GEAR_RATIO = 144 / 11 / 1;

        public static final double TELESCOPIC_CONTROLLER_KD = 0;
        public static final double TELESCOPIC_CONTROLLER_KI = 0;
        public static final double TELESCOPIC_CONTROLLER_KP = 0;
        /** units: r/s */
        public static final double TELESCOPIC_MOTION_ACCEL = 50;
        /** units: r/s^2 */
        public static final double TELESCOPIC_MOTION_VEL = 70;
        public static final double TELESCOPIC_MOTION_TIMEOUT = 0;
        public static final double TELESCOPIC_RESET = 0;
        public static final double TELESCOPIC_TOLERANCE = 1;

    }

    public static final class LEDConstants {
        public static final int BUFFER_LENGTH = 60; // TODO: set led length
        public static final int CANdleID = 43; // TODO: config
    }

    public class LLConstants {
        public static String name = "limelight";
        public static double height = 0;
        public static double Pitch = 0; // vertical angle
    }
}