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
import frc.robot.commands.arm.ArmClosedLoop;
import frc.robot.commands.arm.ArmOpenLoop;
import frc.robot.commands.auto.DoNothing;
import frc.robot.commands.leds.Party;
import frc.robot.commands.shooter.SFeederCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.swerve.DriveByJoystick;
import frc.robot.commands.telescopic.TelescopicDeneme;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.team6014.lib.auto.ARCTrajectory;

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
        //private final TelescopicSubsystem mTelescopic = TelescopicSubsystem.getInstance();
        // private final ArmSubsystem mArm = ArmSubsystem.getInstance();
        //private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
        //private final ShooterSubsystem mShooter = ShooterSubsystem.getInstance();

        // controllers
        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);

        // auto
        private final ARCTrajectory trajectories = new ARCTrajectory();
        private SendableChooser<Command> autoChooser;

        // commands
        private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> mDriver.getLeftY(),
                        () -> mDriver.getLeftX(),
                        () -> mDriver.getRawAxis(2),
                        () -> mDriver.R2().getAsBoolean(),
                        () -> mDriver.L1().getAsBoolean(),
                        () -> mDriver.R1().getAsBoolean());

        //private final TelescopicDeneme telescopic = new TelescopicDeneme(() -> mOperator.getRightY());
        //private final ArmOpenLoop armOpenLoop = new ArmOpenLoop(mArm, () -> mOperator.getLeftY(),
        //                () -> mOperator.b().getAsBoolean());
        //private final ShooterCommand shooterIdle = new ShooterCommand().withOpenLoop(0.1);

       
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Open loop commands */
                mDrive.setDefaultCommand(driveByJoystick);
                //mTelescopic.setDefaultCommand(telescopic);
                //mArm.setDefaultCommand(armOpenLoop);
                //mShooter.setDefaultCommand(shooterIdle);

                DriverStation.silenceJoystickConnectionWarning(true);
                LiveWindow.disableAllTelemetry();
                LiveWindow.setEnabled(false);

                configureButtonBindings();
                configureNamedCommands();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto ", autoChooser);
                SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());
        }

        /*
         * Named commands
         */
        private void configureNamedCommands() {

                // NamedCommands.registerCommand("prepare & shoot amp",
                //                 new SFeederCommand().andThen(new ShooterCommand().withShooterState(ShooterState.AMP)
                //                                 .withTimeout(1.2)));

                // NamedCommands.registerCommand("prepare & shoot speaker",
                //                 new SFeederCommand().andThen(new ShooterCommand().withShooterState(ShooterState.SPEAKER)
                //                                 .withTimeout(1.2)));
                NamedCommands.registerCommand("DoNothing", new DoNothing().withTimeout(1.0));
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

                //mDriver.circle().onTrue(new AllignWithLL(1));
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                //mOperator.x().onTrue(new Party());

                // ---------------------------- Arm
                // Closed Loop
                //mOperator.a().onTrue(new ArmClosedLoop(mArm, 0, 0, false));

                // ---------------------------- Feeder
                //mOperator.y().onTrue(new SFeederCommand());
                // Feeder arbitrary
                // new SFeederCommand(mOperator.getLeftX());

                // ---------------------------- Shooter
                // Closed Loop
                //mOperator.leftBumper().onTrue(new ShooterCommand().withShooterState(ShooterState.AMP));
                //mOperator.rightBumper().onTrue(new ShooterCommand().withShooterState(ShooterState.SPEAKER));
                //mOperator.leftTrigger().onTrue(new ShooterCommand().withShooterState(ShooterState.CLOSED));
                // shooter arbitrary
                // new ShooterCommand().withOpenLoop(mOperator.getLeftX());


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