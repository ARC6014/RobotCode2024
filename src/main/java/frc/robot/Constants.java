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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;
import frc.team6014.lib.util.SwerveUtils.SwerveDriveConstants;

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
public class Constants {
        public static final String CANIVORE_CANBUS = "CANivore";
        public static final String RIO_CANBUS = "rio";
        public static final boolean isTuning = false; // tuning mode for tunable numbers
        public static final boolean DEVELOPER_LOGGING = false;

        public static int Pigeon2CanID = 60;

        public static final double wheelBaseLength = 0.56665;
        private static final double wheelBaseWidth = 0.56665;

        public static final double drivebaseRadius = Math.hypot(wheelBaseWidth / 2.0, wheelBaseLength / 2.0);
        public static final double maxModuleSpeed = 5.0;

        public static final HolonomicPathFollowerConfig holonomicPoseConfig = new HolonomicPathFollowerConfig(
                        new PIDConstants(4.7, 0, 0),
                        new PIDConstants(4.5, 0, 0),
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

        public static final class DriveConstants {
                public static final boolean isFieldOriented = true;
                public static final boolean invertGyro = true; // !! CCW+

                public static final int angleContinuousCurrentLimit = 20;
                public static final int anglePeakCurrentLimit = 35;
                public static final double anglePeakCurrentDuration = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveContinuousCurrentLimit = 30;
                public static final int drivePeakCurrentLimit = 55;
                public static final double drivePeakCurrentDuration = 0.2;
                public static final boolean driveEnableCurrentLimit = true;

                public static final NeutralMode angleMotorNeutralMode = NeutralMode.Coast;
                public static final NeutralMode driveMotorNeutralMode = NeutralMode.Brake;

                public static final double openLoopRamp = 0;
                public static final double closedLoopRamp = 0;

                public static final double drivePowerScalar = 0.95;
                public static final double driveSlewRateLimitX = 7;
                public static final double driveSlewRateLimitY = 7;
                public static final double driveSlewRateLimitRot = 12;

                public static final double angleGearboxRatio = 22.93;
                public static final double driveGearboxRatio = 6.59340659;
                public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

                // PID and Feedforward
                public static final double drivekP = 0.1;
                public static final double drivekI = 0;
                public static final double drivekD = 0;
                public static final double drivekS = 0.016;
                public static final double drivekV = 0.19;
                public static final double drivekA = 0.05;

                public static final double anglekP = 0.27;
                public static final double anglekI = 0;
                public static final double anglekD = 0.0;

                // snap PID is not used currently
                public static final double snapkP = 2.0;
                public static final double snapkI = 0.0;
                public static final double snapkD = 0.0;

                public static final double maxSpeed = 5;

                public static final double maxTransSpeedMetersPerSecond = 3.3;
                public static final double maxAngularSpeedRadPerSec = 2 * Math.PI;
                public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2);

                public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
                                maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

                private static final double maxTransAccelMetersPerSecSq = 2.5;
                public static final TrapezoidProfile.Constraints transPIDconstraints = new TrapezoidProfile.Constraints(
                                maxTransSpeedMetersPerSecond, maxTransAccelMetersPerSecSq);

                public static SwerveDriveConstants swerveConstants = SwerveDriveConstants.generateSwerveConstants(
                                angleContinuousCurrentLimit,
                                anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit,
                                driveContinuousCurrentLimit,
                                drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp,
                                closedLoopRamp);
        }

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
                public static final double angleOffset = -136.93 - 3;
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

                public static final int MOTOR_ID = 20;
                public static final int BORE_ID = 1;

                public static final Gearbox gearRatio = new Gearbox(1, 115.77);
                public static final boolean ARM_MOTOR_INVERTED = false;

                public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;

                /** unit: rev/s */
                public static final double ARM_VELOCITY = 200;
                /** unit: rev/s^2 */
                public static final double ARM_ACCELERATION = 170;

                public static final double kP = 1.2;
                public static final double kD = 0.0;
                public static final double kI = 0;
                public static final double kS = 0.14;
                public static final double kV = 0;
                public static final double kA = 0;
                public static final double kF = 0.4;

                /** unit: rotations */
                // before arm broke down
                public static final double POSITION_OFFSET_OLD = Conversions.degreesToRevolutions(13.34);
                // final offset used in the regs
                public static final double POSITION_OFFSET = Conversions
                                .degreesToRevolutions(80 - 4 + 2 + 2);

                /** unit: rotations */
                public static final double ANGLE_TOLERANCE = Conversions.degreesToRevolutions(0.5);

                /* Arm angles for setpoints (relative to the zero of Bore) */
                /** unit: degrees */
                public static final double ZERO = 0;
                /** unit: degrees */
                public static final double INTAKE = 10; // 10.5
                /** unit: degrees */
                public static final double SPEAKER_LONG = 54;
                /** unit: degrees */
                public static final double SPEAKER_SHORT = 32; // 22.5 31.5 36
                /** unit: degrees */
                public static final double FROM_INTAKE = 75;
                /** unit: degrees */
                public static final double AMP = 110;
                /** unit degrees */
                public static final double CLIMB = 45;
                /** unit: degrees */
                public static final double NOTE_PASS = 25;

                /** unit: degrees */
                public static final double LAST_RESORT_ANGLE_CUTOFF = 120;

                // interpolation - currently using lookup tables
                // quadratic fit
                public static final double COEFFICIENT_QUADRATIC = -1.770;
                public static final double COEFFICIENT_LINEAR = 17.05;
                public static final double COEFFICIENT_CONSTANT = 21.47;
                public static final boolean IS_ON_FIELD = true;

        }

        public static final class IntakeConstants {
                public static final int RUNNING_MOTOR_ID = 10;
                public static final int BEAM_BREAK_ID = 2;

                /** unit: percent */
                public static final double FORWARD_PERCENT = 8;
                /** unit: percent */
                public static final double REVERSE_PERCENT = -9.75;
                /** unit: percent */
                public static final double FEED_PERCENT = -5;

                /** unit: V/(rev/s) */
                public static final double RUN_kP = 3.1502;
                /** unit: V/((rev/s) * s) */
                public static final double RUN_kI = 0;
                /** unit: Vs/(rev/s) */
                public static final double RUN_kD = 0.821;
                public static final double RUN_kS = 0.3;
                public static final double RUN_kV = 0;

        }

        public static final class WristConstants {
                public static final int ANGLE_MOTOR_ID = 11;
                public static final int BORE_ID = 0;

                public static final Gearbox gearbox = new Gearbox(1, 12);

                /** REV Bore Encoder position offset, unit: revolutions */
                public static final double POSITION_OFFSET = Conversions
                                .degreesToRevolutions(156 - 3);
                /** REV Bore Encoder position, with the closed position as 0, unit: degrees */
                public static final double OPEN_POSITION = 152;
                /** REV Bore Encoder position, with the closed position as 0, unit: degrees */
                public static final double CLOSED_POSITION = 3.2;
                /** unit: degrees */
                public static final double STOP_POSITION = 0;

                /** unit: revolutions */
                public static final double POSITION_EQUALITY_TOLERANCE = Conversions.degreesToRevolutions(4);

                /** unit: V/rev */
                public static final double ANGLE_kP = 1.2;
                /** unit: V/(rev * s) */
                public static final double ANGLE_kI = 0;
                /** unit: Vs/rev */
                public static final double ANGLE_kD = 0.09;

                public static final double ANGLE_kS = 0;

                public static final double ANGLE_kV = 0;

                /** unit: rev/s */
                public static final double WRIST_VELOCITY = 150;
                /** unit: rev/s^2 */
                public static final double WRIST_ACCELERATION = 300;

                public static final int STATOR_CURRENT_LIMIT = 80;

        }

        public static final class ShooterConstants {

                /* Motors & Sensors */
                public static final int MASTER_MOTOR_ID = 30;
                public static final int SLAVE_MOTOR_ID = 31;
                public static final int FEEDER_MOTOR_ID = 32;

                public static final double ShooterOfsett = 2;

                public static final int BEAM_BREAK_ID = 3;

                public static final double kMinOutput = -1;
                public static final double kMaxOutput = 1;

                /* PID & FF */
                // NOT USED
                public static final double kP = 0.000594;
                public static final double kD = 0.132;
                public static final double kI = 0;
                public static final double kIz = 0;
                public static final double kFF = 0;

                /* Neutral Modes */
                public static final IdleMode FEEDER_MODE = IdleMode.kBrake;
                public static final IdleMode MASTER_MODE = IdleMode.kBrake;

                /* Inverts */
                public static final boolean MASTER_INVERTED = false;
                public static final boolean SLAVE_INVERTED = false;
                public static final boolean FEEDER_INVERTED = false;

                /* VOLTAGE */
                // any voltage should not exceed 11V
                public static final double AMP_VOLTAGE = 5;
                public static final double SPEAKER_SHORT_VOLTAGE = 9.75;
                public static final double SPEAKER_LONG_VOLTAGE = 9.75;
                public static final double INTAKE_FROM_SOURCE = -6;

                public static final double PASS_NOTE = 7.0;

                public static final double FEEDER_OUT = 8;
                public static final double FEEDER_AMP = 7.5;
                public static final double FEEDER_FROM_INTAKE = 4;
                public static final double FEEDER_REVERSE = -6.31;

                // voltage-control
                public static final boolean IS_VOLTAGE_MODE = false;

                // scalar to multiply lower set of shooter
                public static final double SLAVE_FUDGE_FACTOR = 1.0517;

        }

        public static final class TelescopicConstants {
                public static final int MASTER_MOTOR_ID = 17;
                public static final int SLAVE_MOTOR_ID = 18;

                public static final double TELESCOPIC_GEAR_RATIO = 13.08;

                public static final double TELESCOPIC_CONTROLLER_KP = 0.5;
                public static final double TELESCOPIC_CONTROLLER_KI = 0;
                public static final double TELESCOPIC_CONTROLLER_KD = 0;

                /** units: r/s */
                public static final double TELESCOPIC_MOTION_VEL = 70;
                /** units: r/s^2 */
                public static final double TELESCOPIC_MOTION_ACCEL = 50;
                public static final double TELESCOPIC_MOTION_TIMEOUT = 3;
                public static final double TELESCOPIC_RESET = 0;
                public static final double TELESCOPIC_TOLERANCE = 2;
                public static final boolean IS_INVERTED = true;
                public static final double DENEME = 0.5;
                /** units: cm */
                public static final double SPROCKET_CIRCUMFERENCE = 2.5 * Math.PI;
                /** units: cm */
                public static final double MAX_EXTENSION = 25.7; // theoretical max 34 cm
                /** units: cm */
                public static final double MAX_RETRACTION = 1.5;
                /** units: A */
                public static final int STATOR_CURRENT_LIMIT = 80;
        }

        public static final class LEDConstants {
                public static final int BUFFER_LENGTH = 30;
                public static final int PWM_PORT = 0;
                public static final int CANDLE_ID = 43;
        }

        public static class LLConstants {
                public static String name = "limelight";
                public static double[] camPose_RobotField = { 0, 0, 0, 0, 0, 0 };// x, y, z, roll, pitch, yaw
        }

        public static class PVConstants {
                public static final String PV_CAMERA_NAME = "photonvision";
                public static final String APRIL_TAG_LAYOUT = "tagMap.json";
                public static final double[][] CAM_POSE = { { 0, 0, 0 }, { 0, 0, 0 } };// x, y, z, roll, pitch, yaw

        }

        public static class Characterization {
                public static final double START_DELAY_SECS = .5;
                public static final double RAMP_VOLTS_PER_SEC = 0.5;
        }

        public static final class FieldConstants {

                public static final double FieldX = Units.inchesToMeters(651.223);
                public static final double FieldY = Units.inchesToMeters(323.277);
                public static final double WING_X = Units.inchesToMeters(229.201);
                public static final double PODIUM_X = Units.inchesToMeters(126.75);
                public static final double STARTING_LINE_X = Units.inchesToMeters(74.111);

                public static final Pose2d BLUE_SPEAKER = new Pose2d(
                                0,
                                Units.inchesToMeters(218.416),
                                new Rotation2d(Math.PI));
                public static final Pose2d BLUE_SPEAKER_CENTER = new Pose2d(
                                BLUE_SPEAKER.getX(),
                                BLUE_SPEAKER.getY() - 0.15,
                                BLUE_SPEAKER.getRotation());
                public static final Pose2d BLUE_AMP = new Pose2d(
                                (126 + 30.5) / 100,
                                Units.inchesToMeters(322.996),
                                Rotation2d.fromDegrees(-90));
                public static final Pose2d BLUE_SOURCE = new Pose2d(FieldX - Units.inchesToMeters(76.1) / 2,
                                Units.inchesToMeters(60.75), Rotation2d.fromDegrees(120));

                public static final Pose2d RED_SPEAKER = new Pose2d(
                                FieldX - 0.1,
                                BLUE_SPEAKER.getY(),
                                BLUE_SPEAKER.getRotation().unaryMinus());
                public static final Pose2d RED_SPEAKER_CENTER = new Pose2d(
                                RED_SPEAKER.getX(),
                                RED_SPEAKER.getY() + 0.22,
                                RED_SPEAKER.getRotation());
                public static final Pose2d RED_AMP = new Pose2d(
                                FieldX - BLUE_AMP.getX(),
                                BLUE_AMP.getY(),
                                BLUE_AMP.getRotation());
                public static final Pose2d RED_SOURCE = new Pose2d(
                                FieldX - BLUE_AMP.getX(),
                                BLUE_AMP.getY(),
                                Rotation2d.fromDegrees(180 - BLUE_SOURCE.getRotation().getDegrees()));

                public static final Translation3d BLUE_TOP_RIGHT_SPEAKER = new Translation3d(
                                Units.inchesToMeters(18.055),
                                Units.inchesToMeters(238.815),
                                Units.inchesToMeters(13.091));

                public static final Translation3d BLUE_TOP_LEFT_SPEAKER = new Translation3d(
                                Units.inchesToMeters(18.055),
                                Units.inchesToMeters(197.765),
                                Units.inchesToMeters(83.091));

                public static final Pose2d[] NOTE_POSITIONS = new Pose2d[] {
                                new Pose2d(2.90, 6.68, Rotation2d.fromDegrees(0)),
                                new Pose2d(2.90, 5.55, Rotation2d.fromDegrees(0)),
                                new Pose2d(2.90, 4.09, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.29, 7.44, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.29, 5.78, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.29, 4.12, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.29, 2.45, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.29, 0.77, Rotation2d.fromDegrees(0)),
                };

                public static final Pose2d[] START_POSITIONS = new Pose2d[] {
                                new Pose2d(0.73, 6.74, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.51, 5.57, Rotation2d.fromDegrees(0)),
                                new Pose2d(0.73, 4.43, Rotation2d.fromDegrees(0)),
                                new Pose2d(0.73, 3.25, Rotation2d.fromDegrees(0)),
                                new Pose2d(0.73, 2.27, Rotation2d.fromDegrees(0)),
                };

                // Setpoints for LookUp Table interpolation
                /** Distance(m), Arm Angle(degrees), Shooter Speed(volts) */
                public static final double[][] SHOOT_POSITIONS = {
                                { 1.44, 37.5 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 1.69, 40 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 1.94, 42 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 2.19, 44.5 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 2.44, 48.25 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 2.60, 51 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 2.69, 51 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 2.94, 53.75 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 3.00, 53.9 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 3.19, 55.5 - ShooterConstants.ShooterOfsett, 9.75 },

                                { 3.56, 56 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 3.69, 58.5 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 3.94, 59 - ShooterConstants.ShooterOfsett, 9.75 },
                                { 4.19, 60.5 - ShooterConstants.ShooterOfsett, 10 },
                                { 4.44, 61 - ShooterConstants.ShooterOfsett, 10 },
                                { 4.69, 61 - ShooterConstants.ShooterOfsett, 10.5 },
                                { 4.94, 61.85 - ShooterConstants.ShooterOfsett, 10.5 },
                                { 6.0, 62.2 - ShooterConstants.ShooterOfsett, 11 }
                };
        }
}