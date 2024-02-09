// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.music.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.AllignWithLL;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.arm.ArmOpenLoop;
import frc.robot.commands.arm.ArmStateSet;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.auto.FakeIntake;
import frc.robot.commands.auto.FakeShoot;
import frc.robot.commands.intake.IntakeOpenLoop;
import frc.robot.commands.intake.IntakeSetState;
import frc.robot.commands.intake.WristOpenLoop;
import frc.robot.commands.intake.WristSetState;
import frc.robot.commands.leds.Party;
import frc.robot.commands.shooter.SFeederCommand;
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
        // public static Orchestra mOrchestra = new Orchestra();

        /* CONTROLLERS */
        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);
        private final CommandPS4Controller mOperator2 = new CommandPS4Controller(2);

        /* AUTO */
        private final ARCTrajectory trajectories = new ARCTrajectory();
        private SendableChooser<Command> autoChooser;

        /* COMMANDS */
        private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> mDriver.getLeftY(),
                        () -> mDriver.getLeftX(),
                        () -> -mDriver.getRightX(),
                        () -> mDriver.R2().getAsBoolean(),
                        () -> mDriver.L1().getAsBoolean(),
                        () -> mDriver.R1().getAsBoolean());

        // private final TelescopicOpenLoop telescopicOpenLoop = new
        // TelescopicOpenLoop(mTelesopic, () -> mOperator.getRightY());
        private final ArmOpenLoop armOpenLoop = new ArmOpenLoop(mArm, () -> -mOperator.getLeftY());
        // private final ShooterCommand shooterOpenLoop = new ShooterCommand().withOpenLoop(mOperator.getLeftY());

        private final WristOpenLoop wristOpenLoop = new WristOpenLoop(mWrist, () -> mOperator2.getLeftX());
        private final IntakeOpenLoop intakeOpenLoop = new IntakeOpenLoop(mIntake, () -> mOperator2.getRightX());

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Open loop commands */
                mDrive.setDefaultCommand(driveByJoystick);
                // mTelescopic.setDefaultCommand(telescopicOpenLoop);
                mWrist.setDefaultCommand(wristOpenLoop);
                mIntake.setDefaultCommand(intakeOpenLoop);
                // mShooter.setDefaultCommand(shooterOpenLoop);

                DriverStation.silenceJoystickConnectionWarning(true);
                LiveWindow.disableAllTelemetry();
                LiveWindow.setEnabled(false);

                configureNamedCommands();
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto ", autoChooser);
                SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
        }

        /*
         * Named commands
         */
        private void configureNamedCommands() {

                // NamedCommands.registerCommand("prepare & shoot amp",
                // new SFeederCommand().andThen(new
                // ShooterCommand().withShooterState(ShooterState.AMP)
                // .withTimeout(1.2)));

                // NamedCommands.registerCommand("prepare & shoot speaker",
                // new SFeederCommand().andThen(new
                // ShooterCommand().withShooterState(ShooterState.SPEAKER)
                // .withTimeout(1.2)));
                NamedCommands.registerCommand("Shoot", new FakeShoot().withTimeout(1.0));
                NamedCommands.registerCommand("Intake", new FakeIntake().withTimeout(0.5));
                NamedCommands.registerCommand("Field-Oriented Turn (-45)", new FieldOrientedTurn(mDrive, -45));
                NamedCommands.registerCommand("Field-Oriented Turn (45)",
                                new FieldOrientedTurn(mDrive, 45).withTimeout(1));
                NamedCommands.registerCommand("DoNothing", new DoNothing());

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
                // mDriver.cross().onTrue(mDrive.orchestraCommand());
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                /* ARM */
                mOperator.leftBumper().toggleOnTrue(new ArmStateSet(mArm,
                                ArmControlState.ZERO));
                mOperator.povDown().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.INTAKE));
                mOperator.povLeft().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.AMP));
                mOperator.povRight().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.SPEAKER_SHORT));
                mOperator.povUp().toggleOnTrue(new ArmStateSet(mArm, ArmControlState.SPEAKER_LONG));
                mOperator.rightBumper().onTrue(armOpenLoop);

                /* SHOOTER */
                mOperator.x().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.AMP));
                mOperator.y().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER));
                mOperator.a().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.CLOSED));
                mOperator.b().whileTrue(new SFeederCommand(0.8));
                mOperator.rightTrigger().toggleOnTrue(new ShooterCommand().withShooterState(ShooterState.SMART_VOLTAGE));

                /* WRIST */
                mOperator2.povDown().toggleOnTrue(new WristSetState(mWrist, Position.CLOSED));
                mOperator2.povUp().toggleOnTrue(new WristSetState(mWrist, Position.OPEN));
                mOperator2.R1().onTrue(wristOpenLoop);

                /* INTAKE */
                mOperator2.circle().toggleOnTrue(new IntakeSetState(mIntake, Running.FORWARD));
                mOperator2.cross().toggleOnTrue(new IntakeSetState(mIntake, Running.REVERSE));

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