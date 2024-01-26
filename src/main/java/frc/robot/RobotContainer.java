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
import frc.robot.commands.auto.ARCTrajectory;
import frc.robot.commands.leds.Party;
import frc.robot.commands.swerve.DriveByJoystick;
import frc.robot.commands.telescopic.TelescopicDeneme;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;

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
        private final IntakeSubsystem mIntake = IntakeSubsystem.getInstance();

        // controllers
        private final CommandPS4Controller mDriver = new CommandPS4Controller(0);
        private final CommandXboxController mOperator = new CommandXboxController(1);

        // auto
        private final ARCTrajectory trajectories = new ARCTrajectory();
        private SendableChooser<Command> autoChooser;

        // commands
        private final DriveByJoystick driveByJoystick = new DriveByJoystick(() -> mDriver.getLeftY() * -1,
                        () -> mDriver.getLeftX() * -1,
                        () -> mDriver.getRawAxis(2),
                        () -> mDriver.R2().getAsBoolean(),
                        () -> mDriver.L1().getAsBoolean(),
                        () -> mDriver.R1().getAsBoolean());

        private final TelescopicDeneme telescopic = new TelescopicDeneme(() -> mOperator.getRightY());
        private final ArmOpenLoop armOpenLoop = new ArmOpenLoop(mArm, () -> mOperator.getLeftY(),
                        () -> mOperator.b().getAsBoolean());

        com.ctre.phoenix6.Orchestra mOrchestraV6 = new com.ctre.phoenix6.Orchestra();
        Orchestra mOrchestra = new Orchestra();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                /* Open loop commands */
                mDrive.setDefaultCommand(driveByJoystick);
                mTelescopic.setDefaultCommand(telescopic);
                mArm.setDefaultCommand(armOpenLoop);

                DriverStation.silenceJoystickConnectionWarning(true);
                LiveWindow.disableAllTelemetry();
                LiveWindow.setEnabled(false);

                configureNamedCommands();

                // Configure the button bindings
                configureButtonBindings();

                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto ", autoChooser);
                SmartDashboard.putBoolean("Is AutoBuilder Configured", AutoBuilder.isConfigured());

                carpeDiemV5();
                // carpeDiem();
        }

        /*
         * Named commands
         */
        private void configureNamedCommands() {
                // NamedCommands.registerCommand(null, driveByJoystick);
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
                mDriver.circle().onTrue(new AllignWithLL(1)); // ID should change
                mDriver.cross().onTrue(new ResetGyro(mDrive));

                mOperator.x().onTrue(new Party());

                // Arm Closed Loop
                mOperator.a().onTrue(new ArmClosedLoop(mArm, 0, 0, false, ArmConstants.armCruiseVelocity,
                                ArmConstants.armAcceleration));

                // Orchestra open/pause/stop

                mDriver.options().onTrue(new Command() {
                        @Override
                        public void initialize() {
                                mOrchestra.play();
                        }
                });

                mDriver.share().onTrue(new Command() {
                        @Override
                        public void initialize() {
                                mOrchestra.pause();
                        }
                });

                // mDriver.options().onTrue(new Command() {
                // @Override
                // public void initialize() {
                // mOrchestraV6.play();
                // }
                // });

                // mDriver.share().onTrue(new Command() {
                // @Override
                // public void initialize() {
                // mOrchestraV6.pause();
                // }
                // });

        }

        /*
         * Talon FX Music
         * Let's Have some fun
         */
        private void carpeDiem() {
                mOrchestraV6.addInstrument(mIntake.getMotors().get(0));
                mOrchestraV6.addInstrument(mIntake.getMotors().get(1));
                mOrchestraV6.loadMusic("Never-Gonna-Give-You-Up-3.chrp");
                mOrchestraV6.play();
        }

        /*
         * Carpediem function WPI_TALON_FX For V5
         */
        private void carpeDiemV5() {
                for (var motor : mDrive.getDriveMotors()) {
                        mOrchestra.addInstrument(motor);
                }
                mOrchestra.loadMusic("Never-Gonna-Give-You-Up-3.chrp");
                mOrchestra.play();
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