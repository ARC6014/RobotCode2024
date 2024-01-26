// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
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
public final class Constants {
    public static final String CANIVORE_CANBUS = "CANivore"; 
    public static final String RIO_CANBUS = "rio";
    public static final int Pigeon2CanID = 50; 
    public static final boolean tuningMode = false;

    public static final double wheelBaseLength = 0.690; // TODO: Config
    private static final double wheelBaseWidth = 0.690; // TODO: Config
    
    public static final double drivebaseRadius = Math.hypot(wheelBaseWidth / 2.0, wheelBaseLength / 2.0);
    public static final double maxModuleSpeed = 3.0;

    // TODO: Tune PID
    public static final HolonomicPathFollowerConfig holonomicPoseConfig = new HolonomicPathFollowerConfig(new PIDConstants(0, 0, 0), new PIDConstants(0, 0, 0), maxModuleSpeed, drivebaseRadius, new ReplanningConfig());

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
        // TODO: Config gyro
        public static final boolean invertGyro = false; // !! CCW+

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

        public static final double angleGearboxRatio = 22.93; // TODO: Config
        public static final double driveGearboxRatio = 6.59340659; // TODO: Config
        public static final double wheelCircumference = Units.inchesToMeters(4) * Math.PI;

        // PID and Feedforward
        public static final double drivekP = 0.05;
        public static final double drivekI = 0;
        public static final double drivekD = 0;
        public static final double drivekS = 0.016;
        public static final double drivekV = 0.19;
        public static final double drivekA = 0.0;

        public static final double anglekP = 0.27;
        public static final double anglekI = 0;
        public static final double anglekD = 0.0;

        public static final double snapkP = 2.5;
        public static final double snapkI = 0.0;
        public static final double snapkD = 0.01;

        public static final double maxSpeed = 5;

        public static final double maxTransSpeedMetersPerSecond = 3.3; // translation speed (x/y)
        public static final double maxAngularSpeedRadPerSec = 2 * Math.PI; // angular speed (omega)
        public static final double maxAngularAccelRadPerSecSq = Math.pow(maxAngularSpeedRadPerSec, 2); // angular acceleration

        public static final TrapezoidProfile.Constraints rotPIDconstraints = new TrapezoidProfile.Constraints(
                maxAngularSpeedRadPerSec, maxAngularAccelRadPerSecSq);

        // added these two for LLalignment not sure if it makes sense
        private static final double maxTransAccelMetersPerSecSq = 2; 
        public static final TrapezoidProfile.Constraints transPIDconstraints = new TrapezoidProfile.Constraints(maxTransSpeedMetersPerSecond, maxTransAccelMetersPerSecSq);

        public static SwerveDriveConstants swerveConstants = SwerveDriveConstants.generateSwerveConstants(
                angleContinuousCurrentLimit,
                anglePeakCurrentLimit, anglePeakCurrentDuration, angleEnableCurrentLimit, driveContinuousCurrentLimit,
                drivePeakCurrentLimit, drivePeakCurrentDuration, driveEnableCurrentLimit, openLoopRamp, closedLoopRamp);
    }

    // TODO: Configure each module's angle offset in calibration
    public static final class SwerveModuleFrontLeft {
        public static final int angleMotorID = 10;
        public static final int driveMotorID = 11;
        public static final int cancoderID = 01;
        public static final double angleOffset = -206.63 - 45 - 17 + 180 + 3; // added 3
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleFrontRight {
        public static final int angleMotorID = 16;
        public static final int driveMotorID = 17;
        public static final int cancoderID = 02;
        public static final double angleOffset = -152.06 - 12.48 + 2; // added 2
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearLeft {
        public static final int angleMotorID = 14;
        public static final int driveMotorID = 15;
        public static final int cancoderID = 03;
        public static final double angleOffset = -22.06 + 180 -115 - 50; // -50
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }

    public static final class SwerveModuleRearRight {
        public static final int angleMotorID = 12;
        public static final int driveMotorID = 13;
        public static final int cancoderID = 04;
        public static final double angleOffset = -1.49 + 109.4;
        public static final double modulekS = DriveConstants.drivekS;
        public static final double modulekV = DriveConstants.drivekV;
    }
  
    public static final class ArmConstants {

        public static final int motorID = 30; // TODO: Config
        public static final int boreChannel = 0; // TODO: Config
        public static final Gearbox gearRatio = new Gearbox(119.5 / 1); // TODO: Config
        public static final boolean motorInverted = false; // TODO: Config

        public static final double bottomSoftLimit = -25; // TODO: Config
        public static final double topSoftLimit = 232; // TODO: Config

        public static final double armCruiseVelocity = 1000; // Mind units!
        public static final double armAcceleration = 2000; // Mind units!

        public static final double rampRate = 0.09;
        public static final double kP = 0.085; // TODO: Config
        public static final double kD = 4.5; // TODO: Config
        public static final double kI = 0; // TODO: Config
        public static final double kG = 0;


        public static final double resetAngle = 0; // TODO: Config
        public static final double positionOffset = 0; // TODO: Config starting position offset of bore
        public static final double angleTolerance = 1.5;

        /* Arm angles for setpoints */
        public static final double ZERO = 90;
        public static final double INTAKE = 75;
        public static final double SPEAKER = 56;
        public static final double AMP = -45; // Double check negative
        public static final double distancePerRotation = 0;
  
    }

    public static final class IntakeConstants {
        // TODO: determine actual ids
        public static final int runningMotorId = 20;
        public static final int angleMotorId = 21;

        public static final int boreEncoderDioId = 0;
        public static final int beamBreakSensorDioId = 0;

        public static final double positionOffset = 0;

        /** REV Bore Encoder position, with the horizontal as 0, unit: revolutions */
        public static final double openPosition = 0;
        /** REV Bore Encoder position, with the horizontal as 0, unit: revolutions */
        public static final double closedPosition = 0;

        public static final double forwardVelocity = 0;
        public static final double reverseVelocity = 0;

        // TODO: equality tolerances
        public static final double positionEqualityTolerance = 0;
        public static final double velocityEqualityTolerance = 0;

        /** unit: V/rev */
        public static final double ANGLE_kP = 0;
        /** unit: V/(rev * s) */
        public static final double ANGLE_kI = 0;
        /** unit: Vs/rev */
        public static final double ANGLE_kD = 0;

        /** unit: V/(rev/s) */
        public static final double RUN_kP = 0;
        /** unit: V/((rev/s) * s) */
        public static final double RUN_kI = 0;
        /** unit: Vs/(rev/s) */
        public static final double RUN_kD = 0;

        // TODO: voltage cutoff
        public static final double maxVoltageCutoff = 0;

        public static final double kG = 0;

        /** unit: revolutions */
        public static final double stopPosition = 0;

        public static final Gearbox gearbox = new Gearbox(72, 1);
    }

    public static final class ShooterConstants {

		public static final int MASTER_MOTOR_ID = 40;
        public static final int SLAVE_MOTOR_ID = 41;
        public static final int FEEDER_MOTOR_ID = 42;
		public static final double kP = 0;
        public static final double maxRPM = 0;
        public static final double kMinOutput = 0;
        public static final double kMaxOutput = 0;
        public static final double kFF = 0;
        public static final double kIz = 0;
        public static final double kD = 0;
        public static final double kI = 0;
        
        public static final IdleMode FEEDER_MODE = IdleMode.kBrake;
		public static final IdleMode MASTER_MODE = IdleMode.kBrake;
		public static final int BEAM_BREAK_ID = 0;
        public static final double AMP_SHOOT_RPM = 0;
        public static final double SPEAKER_SHOOT_RPM = 0;
    }

    public static final class TelescopicConstants {
        public static final int MASTER_MOTOR_ID = 51; // TODO: Config
        public static final int SLAVE_MOTOR_ID = 52; // TODO: Config

        public static final int TELESCOPIC_GEAR_RATIO = 144 / 11 / 1;

        public static final double TELESCOPIC_CONTROLLER_KD = 0;
        public static final double TELESCOPIC_CONTROLLER_KI = 0;
        public static final double TELESCOPIC_CONTROLLER_KP = 0;
        public static final double TELESCOPIC_MOTION_ACCEL = 200;
        public static final double TELESCOPIC_MOTION_VEL = 500;
        public static final double TELESCOPIC_MOTION_TIMEOUT = 0;
        public static final double TELESCOPIC_RESET = 0;
        public static final double TELESCOPIC_TOLERANCE = 1;


    }

    public static final class LEDConstants {
        // The length of the buffer in pixels / pixel count of led
        public static final int BUFFER_LENGTH = 60; // TODO: set led length

        public static final double BREATHE_DURATION = 1.0;
        public static final double STROBE_FAST = 0.1;
        public static final double STROBE_SLOW = 0.2;
        /* For CANdle */
        public static final int CANdleID = 53; // TODO: config

        public static final class Colors {
            /* Colors go:
             * Disabled: pink breathe
             * Cone Mode: strobe orange
             * Cube Mode: strobe purple
             */
            public static final Color DISABLED = Color.kPaleVioletRed; // run in disabled init
            public static final Color NO_COLOR = Color.kBlack; // no color basically

            public static final Color CONE = Color.kOrange; // cone mode
            public static final Color CUBE = Color.kPurple; // cube mode

        }
    }

    public class LLConstants{
        public static String name = "limelight";
        public static double height = 0;
        public static double Pitch = 0; // vertical angle
      }
}