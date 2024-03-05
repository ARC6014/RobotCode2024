// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToSpeaker extends Command {
  private final DriveSubsystem mDrive;

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(DriveConstants.anglekP,
      DriveConstants.anglekI,
      DriveConstants.anglekD, DriveConstants.rotPIDconstraints);

  // in Degrees
  private double targetAngle;
  private double tethaSpeed = 0;

  private Pose2d currPose = new Pose2d();

  private final SlewRateLimiter mSlewRot = new SlewRateLimiter(DriveConstants.driveSlewRateLimitRot);

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker(DriveSubsystem mDrive) {
    this.mDrive = mDrive;
    m_thetaController.setTolerance(Math.toRadians(1));
    addRequirements(this.mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currPose = mDrive.getPose();
    m_thetaController.reset(currPose.getRotation().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    targetAngle = calcTargetAngle(currPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currPose = mDrive.getPose();

    tethaSpeed = m_thetaController.calculate(currPose.getRotation().getRadians(),
        Units.degreesToRadians(targetAngle));

    tethaSpeed = mSlewRot.calculate(inputTransform(tethaSpeed) *
        DriveConstants.maxAngularSpeedRadPerSec) * 50.0;

    mDrive.swerveDrive(0, 0, tethaSpeed, true);
    SmartDashboard.putNumber("LL Target Angle", targetAngle);
    SmartDashboard.putBoolean("LL Rotate SP", m_thetaController.atGoal());
    SmartDashboard.putNumber("LL T S", tethaSpeed);
  }

  public double calcTargetAngle(Pose2d currPose) {

    double tAngle = 0;
    double tetha = 0;
    double RobotX = currPose.getX();
    double RobotY = currPose.getY();

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      tetha = Units.radiansToDegrees(Math.acos(RobotX
          / currPose.getTranslation().getDistance(Constants.FieldConstants.BLUE_SPEAKER_CENTER.getTranslation())));
      tAngle = 180 - tetha;
    } else {
      tetha = Units.radiansToDegrees(
          Math.acos((Constants.FieldConstants.FieldX - RobotX)
              / currPose.getTranslation().getDistance(Constants.FieldConstants.RED_SPEAKER_CENTER.getTranslation())));
      tAngle = -tetha;
    }

    if (RobotY < Constants.FieldConstants.BLUE_SPEAKER.getY())
      tAngle = -tAngle;

    return -tAngle;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.swerveDrive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_thetaController.atGoal();
  }

  private double inputTransform(double input) {
    if (input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
  }
}
