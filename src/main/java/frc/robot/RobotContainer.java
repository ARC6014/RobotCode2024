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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.TelescopicConstants;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.idlemodes.SetIdleModeInvert;
import frc.robot.commands.arm.ArmOpenLoop;
import frc.robot.commands.arm.ArmStateSet;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeSetOpenLoop;
import frc.robot.commands.intake.IntakeSetStatePID;
import frc.robot.commands.intake.IntakeStopAtBeambreak;
import frc.robot.commands.intake.WristOpenLoop;
import frc.robot.commands.intake.WristSetState;
import frc.robot.commands.limelight.AllignWithLL;
import frc.robot.commands.limelight.RotateToSpeaker;
import frc.robot.commands.shooter.FeederCommand;
import frc.robot.commands.shooter.FeederStopAtBeambreak;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swerve.DriveByJoystick;
import frc.robot.commands.swerve.FieldOrientedTurn;
import frc.robot.commands.telescopic.TelescopicOpenLoop;
import frc.robot.commands.telescopic.TelescopicStateCommand;
import frc.robot.subsystems.AddressableLEDSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmControlState;
import frc.robot.subsystems.IntakeSubsystem.Running;
import frc.robot.subsystems.ShooterSubsystem.FeederState;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TelescopicSubsystem.TelescopicState;
import frc.robot.subsystems.UsbCam;
import frc.robot.subsystems.WristSubsystem.Position;
import frc.team6014.lib.util.FeedForwardCharacterization;
import io.github.oblarg.oblog.Loggable;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer implements Loggable {
        // The robot's subsystems and commands are defined here...
        private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
        // private final TelescopicSubsystem mTelescopic =
        // TelescopicSubsystem.getInstance();
        private final ArmSubsystem mArm = ArmSubsystem.getInstance();
        private final ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
        private final WristSubsystem mWrist = WristSubsystem.getInstance();
        private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
        private final LimelightSubsystem mLL = LimelightSubsystem.getInstance();

        public static PowerDistribution mPDH = new PowerDistribution();
        private final UsbCam mCamera = new UsbCam();
        // private final AddressableLEDSubsystem mLED = new
        // AddressableLEDSubsystem().getInstance();

        /* CONTROLLERS */
        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);

        /* AUTO */
        private SendableChooser<Command> autoChooser;

        /* COMMANDS */
        // private final TelescopicOpenLoop telescopicOpenLoop = new
        // TelescopicOpenLoop(mTelescopic,
        // () -> mOperator.getRightY());
        private DriveByJoystick driveByJoystick;
        private final ArmOpenLoop armOpenLoop = new ArmOpenLoop(mArm, () -> -mOperator.getLeftY());
        private final WristOpenLoop wristOpenLoop = new WristOpenLoop(mWrist, () -> mOperator.getLeftX());
        private final IntakeOpenLoop intakeOpenLoop = new IntakeOpenLoop(mIntake, () -> mOperator.getRightX());

        // ---------------------- TELEOP COMMANDS ---------------------- //

        private final ParallelCommandGroup openWristStartIntake = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelCommandGroup(
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT)
                                                        .withTimeout(2.25)));

        private final ParallelCommandGroup openWristStartIntakeBeamBreak = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelDeadlineGroup(
                                        new IntakeStopAtBeambreak().withTimeout(3.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN),
                                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT)));

        private final ParallelCommandGroup closeWristStopIntakeArmIntake = new ParallelCommandGroup(
                        new IntakeSetOpenLoop(mIntake, 0.0).withTimeout(0.1),
                        new WristSetState(mWrist, Position.CLOSED),
                        new ArmStateSet(mArm, ArmControlState.INTAKE));

        private final ParallelCommandGroup startStopFeeder = new ParallelCommandGroup(
                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FEED_PERCENT).withTimeout(0.3),
                        new FeederCommand().withFeederState(FeederState.INTAKECEPTION).withTimeout(0.2));

        private final ParallelDeadlineGroup startStopFeederBeamBreak = new ParallelDeadlineGroup(
                        new FeederStopAtBeambreak().withTimeout(2), // this is the deadline
                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FEED_PERCENT),
                        new FeederCommand().withFeederState(FeederState.INTAKECEPTION));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerShort = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.75),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(1.5)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT).withTimeout(1.75));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.POSE_T),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_LONG).withTimeout(1.75));

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
                                        new IntakeStopAtBeambreak().withTimeout(3.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN)),
                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT));

        private final ParallelCommandGroup AUTOopenWristStartIntakeLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.INTAKE),
                        new ParallelDeadlineGroup(
                                        new IntakeStopAtBeambreak().withTimeout(5.0), // this is the deadline
                                        new WristSetState(mWrist, Position.OPEN)),
                        new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT));

        private final ParallelCommandGroup AUTOsetArmFeedAndShootSpeakerShort = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT),
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT).withTimeout(1.75));

        private final ParallelCommandGroup AUTOsetArmFeedAndShootSpeakerLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.POSE_T), // interpolation shooting
                        new SequentialCommandGroup(
                                        new WaitCommand(0.5),
                                        new FeederCommand().withFeederState(FeederState.LET_HIM_COOK)
                                                        .withTimeout(0.5)),
                        new ShooterCommand().withShooterState(ShooterState.SPEAKER_LONG).withTimeout(1.75));

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
                                        () -> mDriver.L1().getAsBoolean(),
                                        () -> mDriver.R1().getAsBoolean());
                } else if (DriverStation.getAlliance().get() == Alliance.Red) {
                        driveByJoystick = new DriveByJoystick(() -> mDriver.getLeftY(),
                                        () -> mDriver.getLeftX(),
                                        () -> -mDriver.getRightX(),
                                        () -> mDriver.R2().getAsBoolean(),
                                        () -> mDriver.L1().getAsBoolean(),
                                        () -> mDriver.R1().getAsBoolean());
                }

                mDrive.setDefaultCommand(driveByJoystick);

                DriverStation.silenceJoystickConnectionWarning(true);
                LiveWindow.disableAllTelemetry();
                LiveWindow.setEnabled(false);

                configureNamedCommands();
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto ", autoChooser);
                SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
                // SmartDashboard.putData("Idle Mode Invert (I-W)", new SetIdleModeInvert());
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

                /* DRIVE */
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /**
                 * MANUAL
                 * METHODS
                 */
                // Shooter - Feeder
                mDriver.square().whileTrue(new FeederCommand().withFeederState(FeederState.LET_HIM_COOK));
                mDriver.circle().whileTrue(new FeederCommand().withFeederState(FeederState.UPSI));
                mDriver.triangle().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT));

                // Arm
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP));
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.POSE_T));

                // Wrist
                mDriver.povLeft().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mDriver.povRight().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));

                mDriver.povUp().onTrue(new RotateToSpeaker());

                // Telescopic
                // mDriver.povDown().whileTrue(new
                // TelescopicStateCommand().withArbitrarySet(TelescopicConstants.DENEME));
                // mDriver.povUp().whileTrue(new
                // TelescopicStateCommand().withTelescopicState(TelescopicState.STOP));
                // mOperator.rightStick().onTrue(telescopicOpenLoop);

                /* COMMAND GROUPS */
                // Intake
                mOperator.rightTrigger().onTrue(openWristStartIntakeBeamBreak);
                // Feed
                mOperator.leftTrigger().onTrue(
                                closeWristStopIntakeArmIntake
                                                .andThen(new WaitCommand(0.5))
                                                .andThen(startStopFeederBeamBreak));

                // Shoot
                mOperator.b().onTrue(setArmFeedAndShootSpeakerShort);
                mOperator.x().onTrue(setArmFeedAndShootAmp);
                mOperator.y().onTrue(setArmFeedAndShootSpeakerLong);

                /* LIMELIGHT */
                mOperator.a().onTrue(new AllignWithLL());

                /* MISC */
                mDriver.touchpad().toggleOnTrue(new SetIdleModeInvert());
                mDriver.L2().toggleOnTrue(new StartEndCommand(() -> DriveSubsystem.getInstance().setSnapActive(true),
                                () -> DriveSubsystem.getInstance().setSnapActive(false)));

        }

        private void configureButtonBindingsAlper() {
                /* DRIVE */
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /* SHOOTER + FEEDER */

                mDriver.square().whileTrue(new FeederCommand().withFeederState(FeederState.LET_HIM_COOK));
                mDriver.circle().whileTrue(new FeederCommand().withFeederState(FeederState.UPSI));

                /* ARM */
                mOperator.leftBumper().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.ZERO));
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP));
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.SPEAKER_LONG));
                // mOperator.rightBumper().onTrue(armOpenLoop);

                /* WRIST */
                mOperator.leftTrigger().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mOperator.rightTrigger().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));
                // mOperator.a().onTrue(wristOpenLoop);

                /* INTAKE */
                mOperator.rightStick().onTrue(intakeOpenLoop);
                mOperator.rightBumper().whileTrue(new IntakeSetOpenLoop(mIntake, IntakeConstants.FORWARD_PERCENT));
                mOperator.leftBumper().whileTrue(new IntakeSetOpenLoop(mIntake, IntakeConstants.REVERSE_PERCENT));
                mOperator.y().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER_LONG));
                mOperator.x().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.AMP));
                mOperator.b().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.CLOSED));
                mOperator.a().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER_SHORT));

                // FeedForwardCharacterization example, use this with any subsystem that you
                // want to characterize
                // mDriver.L2().whileTrue(new FeedForwardCharacterization(mArm,
                // mArm::setArmVoltage, mArm::getCharacterizationVelocity)
                // .beforeStarting(() ->
                // mArm.setArmControlState(ArmControlState.CHARACTERIZATION), mArm));

                mDriver.L2().whileTrue(new StartEndCommand(() -> DriveSubsystem.getInstance().setSnapActive(true),
                                () -> DriveSubsystem.getInstance().setSnapActive(false)));

                mOperator.a().and(mOperator.rightBumper())
                                .whileTrue(mArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                mOperator.b().and(mOperator.rightBumper())
                                .whileTrue(mArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
                mOperator.x().and(mOperator.rightBumper())
                                .whileTrue(mArm.sysIdDynamic(SysIdRoutine.Direction.kForward));
                mOperator.y().and(mOperator.rightBumper())
                                .whileTrue(mArm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }

        /*
         * Named commands
         */
        private void configureNamedCommands() {

                NamedCommands.registerCommand("Field-Oriented Turn (-20)", new FieldOrientedTurn(mDrive, -20));
                NamedCommands.registerCommand("Field-Oriented Turn (25)",
                                new FieldOrientedTurn(mDrive, 25));

                NamedCommands.registerCommand("ReadyIntaking", AUTOopenWristStartIntake);
                NamedCommands.registerCommand("ReadyIntakingLong", AUTOopenWristStartIntakeLong);
                NamedCommands.registerCommand("CloseIntake", closeWristStopIntakeArmIntake);
                NamedCommands.registerCommand("Feed", AUTOstartStopFeederBeamBreak);

                NamedCommands.registerCommand("ShootSpeakerLong", AUTOsetArmFeedAndShootSpeakerLong);
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