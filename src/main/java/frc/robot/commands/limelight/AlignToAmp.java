// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.util.LoggedTunableNumber;

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
  LoggedTunableNumber<Number> scalar = new LoggedTunableNumber<Number>("Align Scalar", 100.0);

  private final SlewRateLimiter mSlewX = new SlewRateLimiter(DriveConstants.driveSlewRateLimitX);
  private final SlewRateLimiter mSlewY = new SlewRateLimiter(DriveConstants.driveSlewRateLimitY);
  private final SlewRateLimiter mSlewRot = new SlewRateLimiter(DriveConstants.driveSlewRateLimitRot);

  /** Creates a new AlignToAmp. */
  public AlignToAmp() {
    x_pid.setTolerance(0.11971);
    y_pid.setTolerance(0.12);
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
      tPose = new Pose2d(new Translation2d(FieldConstants.BLUE_AMP.getX() - .3, FieldConstants.BLUE_AMP.getY() - 0.44),
          Rotation2d.fromDegrees(90));
    } else {
      tPose = new Pose2d(new Translation2d(FieldConstants.RED_AMP.getX() - .3, FieldConstants.RED_AMP.getY() - 0.44),
          Rotation2d.fromDegrees(-90));
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
    xSpeed = mSlewX.calculate(inputTransform(xSpeed) * DriveConstants.maxSpeed) *
        scalar.get().doubleValue();
    ySpeed = mSlewY.calculate(inputTransform(ySpeed) * DriveConstants.maxSpeed) *
        scalar.get().doubleValue();
    tethaSpeed = mSlewRot.calculate(inputTransform(tethaSpeed) *
        DriveConstants.maxAngularSpeedRadPerSec) * scalar.get().doubleValue() / 3.0;

    SmartDashboard.putNumber("Align Amp xSpeed", xSpeed);
    SmartDashboard.putNumber("Align Amp ySpeed", ySpeed);
    SmartDashboard.putString("Align Amp Target", targetPose.toString());

    mDrive.swerveDrive(xSpeed, ySpeed, tethaSpeed, true);
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

  private double inputTransform(double input) {
    if (input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
  }
}
