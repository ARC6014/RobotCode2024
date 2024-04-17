// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.TelescopicConstants;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.idlemodes.SetIdleModeInvert;
import frc.robot.commands.arm.ArmOpenLoop;
import frc.robot.commands.arm.ArmStateSet;
import frc.robot.commands.intake.IntakeSetOpenLoop;
import frc.robot.commands.intake.IntakeStopAtBeambreak;
import frc.robot.commands.intake.WristOpenLoop;
import frc.robot.commands.intake.WristSetState;
import frc.robot.commands.limelight.AlignToAmp;
import frc.robot.commands.limelight.AlignToSourceMid;
import frc.robot.commands.limelight.RotateToSpeaker;
import frc.robot.commands.shooter.FeederCommand;
import frc.robot.commands.shooter.FeederStopAtBeambreak;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swerve.DriveByJoystick;
import frc.robot.commands.telescopic.TelescopicOpenLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmControlState;
import frc.robot.subsystems.ShooterSubsystem.FeederState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.WristSubsystem.Position;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems and commands are defined here...
        private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
        private final TelescopicSubsystem mTelescopic = TelescopicSubsystem.getInstance();
        private final ArmSubsystem mArm = ArmSubsystem.getInstance();
        private final ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
        private final WristSubsystem mWrist = WristSubsystem.getInstance();
        private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
        private final LimelightSubsystem mLL = LimelightSubsystem.getInstance();

        public static PowerDistribution mPDH = new PowerDistribution();
        // private final UsbCam mCamera = new UsbCam();
        // private final AddressableLEDSubsystem mLED = new
        // AddressableLEDSubsystem().getInstance();

        /* CONTROLLERS */
        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);

        /* AUTO */
        private SendableChooser<Command> autoChooser;

        /* COMMANDS */
        private final TelescopicOpenLoop telescopicOpenLoop = new TelescopicOpenLoop(mTelescopic,
                        () -> -mOperator.getLeftY(),
                        () -> -mOperator.getRightY());
        private DriveByJoystick driveByJoystick;

        // ---------------------- TELEOP COMMANDS ---------------------- //

        private final ParallelCommandGroup openWristStartIntake = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelCommandGroup(
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT)
                                                        .withTimeout(2)));

        private final ParallelCommandGroup openWristStartIntakeBeamBreak = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelDeadlineGroup(
                                        new IntakeStopAtBeambreak().withTimeout(2.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT)));

        private final ParallelCommandGroup closeWristStopIntakeArmIntake = new ParallelCommandGroup(
                        new IntakeSetOpenLoop(mIntake, 0.0).withTimeout(0.1),
                        new WristSetState(mWrist, Position.CLOSED),
                        new ArmStateSet(mArm, ArmControlState.INTAKE));

        private final ParallelCommandGroup startStopFeeder = new ParallelCommandGroup(
                        new FeederCommand().withFeederState(FeederState.INTAKECEPTION).withTimeout(0.4),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FEED_PERCENT).withTimeout(0.2)));

        private final ParallelDeadlineGroup startStopFeederBeamBreak = new ParallelDeadlineGroup(
                        new FeederStopAtBeambreak().withTimeout(1.5), // this is the deadline
                        new FeederCommand().withFeederState(FeederState.INTAKECEPTION),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.2),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FEED_PERCENT)));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerShort = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.75),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(1.5)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT).withTimeout(1.75));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerLOOKUP = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.LOOKUP),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.LOOKUP).withTimeout(1.75));

        private final ParallelCommandGroup setArmFeedAndShootAmp = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.AMP),
                        new SequentialCommandGroup(
                                        new WaitCommand(1),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.AMP).withTimeout(2.5));

        // ---------------------- AUTO COMMANDS ---------------------- //

        private final ParallelCommandGroup AUTOopenWristStartIntake = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelDeadlineGroup(
                                        new IntakeStopAtBeambreak().withTimeout(2.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, 10)));

        private final ParallelCommandGroup AUTOopenWristStartIntakeLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelDeadlineGroup(
                                        new IntakeStopAtBeambreak().withTimeout(5.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, 10)));

        // scoring preloaded note - if match ready TURN OFF TUNING MODE!!
        private final ParallelCommandGroup AUTOsetArmFeedAndShootSpeakerShort = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.35),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.2)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT).withTimeout(0.6));

        // interpolation shooting
        private final ParallelCommandGroup AUTOsetArmFeedAndShootSpeakerLongLOOKUP = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.LOOKUP),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.4)),
                        new ShooterCommand().withShooterState(ShooterState.LOOKUP).withTimeout(1.0));

        // setpoint shooting - not tunable
        private final ParallelCommandGroup AUTOsetArmFeedAndShootSpeakerLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_LONG),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.LOOKUP).withTimeout(1.3));

        private final ParallelDeadlineGroup AUTOstartStopFeederBeamBreak = new ParallelDeadlineGroup(
                        new FeederStopAtBeambreak().withTimeout(1), // this is the deadline
                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FEED_PERCENT),
                        new FeederCommand().withFeederState(FeederState.INTAKECEPTION));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Swerve */
                if (DriverStation.getAlliance().get() == Alliance.Blue) {
                        driveByJoystick = new DriveByJoystick(() -> -mDriver.getLeftY(),
                                        () -> -mDriver.getLeftX(),
                                        () -> -mDriver.getRightX(),
                                        () -> mDriver.R2().getAsBoolean(),
                                        () -> mDriver.R1().getAsBoolean(),
                                        () -> mDriver.L1().getAsBoolean());
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                        driveByJoystick = new DriveByJoystick(() -> mDriver.getLeftY(),
                                        () -> mDriver.getLeftX(),
                                        () -> -mDriver.getRightX(),
                                        () -> mDriver.R2().getAsBoolean(),
                                        () -> mDriver.R1().getAsBoolean(),
                                        () -> mDriver.L1().getAsBoolean());
                }

                mDrive.setDefaultCommand(driveByJoystick);
                mTelescopic.setDefaultCommand(telescopicOpenLoop);

                DriverStation.silenceJoystickConnectionWarning(true);
                LiveWindow.disableAllTelemetry();
                LiveWindow.setEnabled(false);

                configureNamedCommands();
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto ", autoChooser);
                SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
         * it to a {@link
         * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {
                // UNUSED:
                // driver: two little buttons near touchpad
                // operator: left stick

                /* DRIVE */
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /**
                 * MANUAL
                 * METHODS
                 */
                // Shooter - Feeder
                mDriver.square().whileTrue(new FeederCommand().withFeederState(FeederState.LET_HIM_COOK));
                mDriver.circle().whileTrue(new FeederCommand().withFeederState(FeederState.UPSI));
                mDriver.triangle().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.LOOKUP));

                // Arm
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP)); // can toggle between AMP
                                                                                              // and CLIMB
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.LOOKUP));

                // Wrist
                mDriver.povLeft().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mDriver.povRight().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));

                // Outtake
                mOperator.leftBumper().whileTrue(new IntakeSetOpenLoop(mIntake, IntakeConstants.REVERSE_PERCENT));

                /* COMMAND GROUPS */
                // Intake
                mOperator.rightTrigger().onTrue(openWristStartIntakeBeamBreak);
                mOperator.rightBumper()
                                .whileTrue(new ShooterCommand().withShooterState(ShooterState.INTAKE_FROM_SOURCE));

                // Feed
                mOperator.leftTrigger().onTrue(
                                closeWristStopIntakeArmIntake
                                                .andThen(new WaitCommand(0.5))
                                                .andThen(startStopFeeder));

                // Shoot
                mOperator.b().onTrue(setArmFeedAndShootSpeakerShort);
                mOperator.x().onTrue(setArmFeedAndShootAmp);
                mOperator.y().onTrue(setArmFeedAndShootSpeakerLOOKUP);
                mOperator.a().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.INTAKE_FROM_SOURCE));
                mOperator.leftBumper().and(mOperator.rightBumper())
                                .toggleOnTrue(new ArmStateSet(mArm, ArmControlState.CLIMB));

                /* LIMELIGHT */
                mDriver.povDown().whileTrue(new AlignToSourceMid());
                mDriver.povUp().whileTrue(new RotateToSpeaker(mDrive));

                /* MISC */
                mDriver.touchpad().toggleOnTrue(new SetIdleModeInvert());
                mDriver.L2().whileTrue(new StartEndCommand(() -> DriveSubsystem.getInstance().setSnapActive(true),
                                () -> DriveSubsystem.getInstance().setSnapActive(false)));

        }

        private void configureButtonBindingsAlper() {

                /* DRIVE */
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /**
                 * MANUAL
                 * METHODS
                 */
                // Shooter - Feeder
                mDriver.square().whileTrue(new FeederCommand().withFeederState(FeederState.LET_HIM_COOK));
                mDriver.circle().whileTrue(new FeederCommand().withFeederState(FeederState.UPSI));
                mDriver.triangle().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.LOOKUP));

                // Arm
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP));
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.LOOKUP));

                // Wrist
                mDriver.povLeft().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mDriver.povRight().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));

                // Intake - outtake
                // mOperator.rightBumper().whileTrue(new IntakeSetOpenLoop(mIntake,
                // IntakeConstants.FORWARD_PERCENT));
                mOperator.leftBumper().whileTrue(new IntakeSetOpenLoop(mIntake, IntakeConstants.REVERSE_PERCENT));

                // Telescopic
                // mDriver.povDown().whileTrue(new
                // TelescopicStateCommand().withArbitrarySet(TelescopicConstants.DENEME));
                // mDriver.povUp().whileTrue(new
                // TelescopicStateCommand().withTelescopicState(TelescopicState.STOP));
                // mOperator.rightStick().onTrue(telescopicOpenLoop);

                /* COMMAND GROUPS */
                // Intake
                mOperator.rightTrigger().onTrue(openWristStartIntakeBeamBreak);
                mOperator.rightBumper().whileTrue(new IntakeSetOpenLoop(mIntake, 0.7));
                mOperator.leftBumper().whileTrue(new IntakeSetOpenLoop(mIntake, -0.7));

                // Feed
                mOperator.leftTrigger().onTrue(
                                closeWristStopIntakeArmIntake
                                                .andThen(new WaitCommand(0.5))
                                                .andThen(startStopFeeder));

                // Shoot
                mOperator.b().onTrue(setArmFeedAndShootSpeakerShort);
                mOperator.x().onTrue(setArmFeedAndShootAmp);
                mOperator.y().onTrue(setArmFeedAndShootSpeakerLOOKUP);
                mOperator.a().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.CLIMB));

                /* LIMELIGHT */
                mDriver.povDown().whileTrue(new AlignToAmp());
                mDriver.povUp().whileTrue(new RotateToSpeaker(mDrive));

                /* MISC */
                mDriver.touchpad().toggleOnTrue(new SetIdleModeInvert());
                mDriver.L2().whileTrue(new StartEndCommand(() -> DriveSubsystem.getInstance().setSnapActive(true),
                                () -> DriveSubsystem.getInstance().setSnapActive(false)));

        }

        /*
         * Named commands
         */
        private void configureNamedCommands() {

                NamedCommands.registerCommand("ReadyIntaking", AUTOopenWristStartIntake);
                NamedCommands.registerCommand("ReadyIntakingLong", AUTOopenWristStartIntakeLong);
                NamedCommands.registerCommand("CloseIntake", closeWristStopIntakeArmIntake);
                NamedCommands.registerCommand("Feed", startStopFeeder);

                NamedCommands.registerCommand("ShootSpeakerLong", AUTOsetArmFeedAndShootSpeakerLongLOOKUP);

                // probably not used
                NamedCommands.registerCommand("ShootSpeakerLongSetpoint", AUTOsetArmFeedAndShootSpeakerLong);

                NamedCommands.registerCommand("ShootSpeakerShort", AUTOsetArmFeedAndShootSpeakerShort);
                NamedCommands.registerCommand("ArmToIntaking", new ArmStateSet(mArm, ArmControlState.INTAKE));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

}