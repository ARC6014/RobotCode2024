// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.AllignWithLL;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetIdleModeInvert;
import frc.robot.commands.arm.ArmOpenLoop;
import frc.robot.commands.arm.ArmStateSet;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.auto.FakeIntake;
import frc.robot.commands.auto.FakeShoot;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeSetOpenLoop;
import frc.robot.commands.intake.IntakeSetStatePID;
import frc.robot.commands.intake.WristOpenLoop;
import frc.robot.commands.intake.WristSetState;
import frc.robot.commands.shooter.SFeederForward;
import frc.robot.commands.shooter.SFeederReverse;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swerve.DriveByJoystick;
import frc.robot.commands.swerve.FieldOrientedTurn;
import frc.robot.commands.telescopic.TelescopicOpenLoop;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmControlState;
import frc.robot.subsystems.IntakeSubsystem.Running;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.WristSubsystem.Position;
import frc.team6014.lib.auto.ARCTrajectory;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.util.LoggedTunableNumber;
import io.github.oblarg.oblog.Loggable;
import frc.team6014.lib.util.FeedForwardCharacterization;

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
        // private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();

        private PowerDistribution mPDH = new PowerDistribution();

        /* CONTROLLERS */

        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);

        /* AUTO */
        private final ARCTrajectory trajectories = new ARCTrajectory();
        private SendableChooser<Command> autoChooser;

        /* COMMANDS */
        private DriveByJoystick driveByJoystick;

        // private final TelescopicOpenLoop telescopicOpenLoop = new
        // TelescopicOpenLoop(mTelesopic, () -> mOperator.getRightY());
        private final ArmOpenLoop armOpenLoop = new ArmOpenLoop(mArm, () -> -mOperator.getLeftY());
        private final WristOpenLoop wristOpenLoop = new WristOpenLoop(mWrist, () -> mOperator.getLeftX());
        // private final IntakeOpenLoop intakeOpenLoop = new IntakeOpenLoop(mIntake, ()
        // -> mOperator.getRightX());

        // private final ParallelCommandGroup openWristStartIntake = new
        // ParallelCommandGroup(
        // new WristSetState(mWrist, Position.OPEN),
        // new IntakeSetOpenLoop(mIntake, Conversions
        // .getSmartVoltage(IntakeConstants.forwardPercent, mPDH.getVoltage()))
        // .withTimeout(2));
        // private final ParallelCommandGroup closeWristStopIntakeArmIntake = new
        // ParallelCommandGroup(
        // new IntakeSetOpenLoop(mIntake, 0.0).withTimeout(0.1),
        // new WristSetState(mWrist, Position.CLOSED),
        // new ArmStateSet(mArm, ArmControlState.INTAKE));

        // private final ParallelCommandGroup startStopFeeder = new
        // ParallelCommandGroup(
        // new IntakeSetOpenLoop(mIntake,
        // Conversions.getSmartVoltage(IntakeConstants.feedPercent, mPDH.getVoltage()))
        // .withTimeout(0.1),
        // new
        // SFeederForward(Conversions.getSmartVoltage(ShooterConstants.FEEDER_FROM_INTAKE,
        // mPDH.getVoltage()))
        // .withTimeout(0.1));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerShort = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT),
                        new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                        new WaitCommand(0.5),
                                                        new SFeederForward(
                                                                        Conversions.getSmartVoltage(
                                                                                        ShooterConstants.FEEDER_OUT,
                                                                                        mPDH.getVoltage()))
                                                                        .withTimeout(0.2)),
                                        new ShooterCommand().withOpenLoop(Conversions
                                                        .getSmartVoltage(ShooterConstants.SPEAKER_SHORT_VOLTAGE,
                                                                        mPDH.getVoltage()))
                                                        .withTimeout(1)));

        // TODO: set rest of the commands to parallel if the above works
        private final ParallelCommandGroup setArmFeedAndShootAmp = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.AMP),
                        new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new SFeederForward(
                                                                        Conversions.getSmartVoltage(
                                                                                        ShooterConstants.FEEDER_OUT,
                                                                                        mPDH.getVoltage()))
                                                                        .withTimeout(1.5)),
                                        new ShooterCommand().withOpenLoop(Conversions
                                                        .getSmartVoltage(ShooterConstants.AMP_VOLTAGE,
                                                                        mPDH.getVoltage()))
                                                        .withTimeout(3)));

        private final ParallelCommandGroup setArmFeedAndShootSpeakerLong = new ParallelCommandGroup(
                        new ArmStateSet(mArm, ArmControlState.SPEAKER_LONG),
                        new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                        new WaitCommand(1),
                                                        new SFeederForward(
                                                                        Conversions.getSmartVoltage(
                                                                                        ShooterConstants.FEEDER_OUT,
                                                                                        mPDH.getVoltage()))
                                                                        .withTimeout(1.5)),
                                        new ShooterCommand().withOpenLoop(Conversions
                                                        .getSmartVoltage(ShooterConstants.SPEAKER_LONG_VOLTAGE,
                                                                        mPDH.getVoltage()))
                                                        .withTimeout(2)));

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Open loop commands */
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

                // mTelescopic.setDefaultCommand(telescopicOpenLoop);
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

        /*
         * Named commands
         */
        private void configureNamedCommands() {

                NamedCommands.registerCommand("Shoot", new FakeShoot().withTimeout(1.0));
                NamedCommands.registerCommand("Intake", new FakeIntake().withTimeout(0.5));
                NamedCommands.registerCommand("Field-Oriented Turn (-20)", new FieldOrientedTurn(mDrive, -20));
                NamedCommands.registerCommand("Field-Oriented Turn (20)",
                                new FieldOrientedTurn(mDrive, 20));
                NamedCommands.registerCommand("DoNothing", new DoNothing());

                // NamedCommands.registerCommand("ReadyIntaking", openWristStartIntake);
                // NamedCommands.registerCommand("CloseIntake", closeWristStopIntakeArmIntake);
                // NamedCommands.registerCommand("Feed", startStopFeeder);
                NamedCommands.registerCommand("ShootSpeakerLong", setArmFeedAndShootSpeakerLong);
                NamedCommands.registerCommand("ShootSpeakerShort", setArmFeedAndShootSpeakerShort);
                NamedCommands.registerCommand("ShootAmp", setArmFeedAndShootAmp);

                NamedCommands.registerCommand("ShooterToIntaking", new ArmStateSet(mArm, ArmControlState.INTAKE));

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

                // new Trigger(() -> mOperator.getRawButton(11)).onTrue(new AllignWithLL(1));
                // new Trigger(() -> mOperator.getRawButton(12)).onTrue(new AllignWithLL(4));

                /* DRIVE */
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /* SHOOTER + FEEDER */
                mDriver.square().whileTrue(new SFeederForward(
                                Conversions.getSmartVoltage(ShooterConstants.FEEDER_OUT, mPDH.getVoltage())));
                mDriver.circle().whileTrue(new SFeederReverse(
                                Conversions.getSmartVoltage(ShooterConstants.FEEDER_REVERSE, mPDH.getVoltage())));

                mDriver.triangle().toggleOnTrue(new ShooterCommand()
                                .withOpenLoop(Conversions.getSmartVoltage(ShooterConstants.SPEAKER_SHORT_VOLTAGE,
                                                mPDH.getVoltage())));

                /* ARM */
                // mOperator.leftBumper().toggleOnTrue(new ArmStateSet(mArm,
                // ArmControlState.ZERO));
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP));
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.SPEAKER_LONG));
                // mOperator.rightBumper().onTrue(armOpenLoop);

                /* WRIST */
                mDriver.povLeft().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mDriver.povRight().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));
                // mOperator.a().onTrue(wristOpenLoop);

                /* INTAKE */
                // mOperator.rightStick().onTrue(intakeOpenLoop);
                // mOperator.rightTrigger().whileTrue(new IntakeSetOpenLoop(mIntake,
                // Conversions.getSmartVoltage(IntakeConstants.forwardPercent,
                // mPDH.getVoltage())));
                // mOperator.leftTrigger().whileTrue(new IntakeSetOpenLoop(mIntake,
                // Conversions.getSmartVoltage(IntakeConstants.reversePercent,
                // mPDH.getVoltage())));

                // mOperator.rightBumper().onTrue(openWristStartIntake);
                // mOperator.leftBumper().onTrue(
                // closeWristStopIntakeArmIntake.andThen(new
                // WaitCommand(1.5)).andThen(startStopFeeder));

                mOperator.y().onTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER));
                mOperator.x().onTrue(new ShooterCommand().withShooterState(ShooterState.AMP));
                mOperator.b().onTrue(new ShooterCommand().withShooterState(ShooterState.CLOSED));

                // FeedForwardCharacterization example, use this with any subsystem that you
                // want to characterize
                mDriver.L2().whileTrue(new FeedForwardCharacterization(mDrive,
                                mDrive::runCharacterizationVolts, mDrive::getCharacterizationVelocity));
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