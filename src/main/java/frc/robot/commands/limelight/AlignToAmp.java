// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import javax.swing.text.TabSet;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToAmp extends Command {
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();

  private double ySpeed = 0, xSpeed = 0, tethaSpeed = 0;

  private final ProfiledPIDController x_pid = new ProfiledPIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD, DriveConstants.transPIDconstraints);
  private final ProfiledPIDController y_pid = new ProfiledPIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD, DriveConstants.transPIDconstraints);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(DriveConstants.anglekP,
      DriveConstants.anglekI,
      DriveConstants.anglekD, DriveConstants.rotPIDconstraints);

  private Pose2d targetPose = new Pose2d();
  private Pose2d currPose = new Pose2d();

  /** Creates a new AlignToAmp. */
  public AlignToAmp() {
    x_pid.setTolerance(0.015);
    y_pid.setTolerance(0.015);
    m_thetaController.setTolerance(Math.toRadians(1));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_pid.reset(mDrive.getPose().getX());
    y_pid.reset(mDrive.getPose().getY());
    m_thetaController.reset(mDrive.getPose().getRotation().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    targetPose = calcTarget();
  }

  private Pose2d calcTarget() {
    Pose2d tPose = new Pose2d();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      tPose = new Pose2d(
          new Translation2d(Constants.FieldConstants.BLUE_AMP.getX(), 0.44),
          new Rotation2d(Units.degreesToRadians(90)));
    } else {
      tPose = new Pose2d(
          new Translation2d(Constants.FieldConstants.RED_AMP.getY(), 0.44),
          new Rotation2d(Units.degreesToRadians(90)));
    }
    return tPose;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = mDrive.getPose();

    xSpeed = x_pid.calculate(currPose.getX(), targetPose.getX());
    ySpeed = y_pid.calculate(currPose.getY(), targetPose.getY());
    tethaSpeed = m_thetaController.calculate(currPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    mDrive.swerveDrive(xSpeed, ySpeed, 0, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.swerveDrive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x_pid.atGoal() && y_pid.atGoal() && m_thetaController.atGoal();
  }
}
