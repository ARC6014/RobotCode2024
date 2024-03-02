// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class RotateToSpeaker extends Command {
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(DriveConstants.anglekP,
      DriveConstants.anglekI,
      DriveConstants.anglekD, DriveConstants.rotPIDconstraints);
  private double tethaSpeed = 0;

  private double targetAngle;

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker() {
    m_thetaController.setTolerance(Math.toRadians(1));
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_thetaController.reset(mDrive.getPose().getRotation().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
    targetAngle = calcTargetAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tethaSpeed = m_thetaController.calculate(mDrive.getPose().getRotation().getRadians(),
        Units.degreesToRadians(targetAngle));
    mDrive.swerveDrive(0, 0, tethaSpeed, true);
  }

  public double calcTargetAngle() {
    double tAngle = mDrive.getPose().getRotation().getDegrees();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      tAngle = 180 -
          Math.acos(mDrive.getPose().getX() /
              mDrive.getPose().getTranslation().getDistance(Constants.FieldConstants.BLUE_SPEAKER.getTranslation()));
    }
    else {
        tAngle = Math.acos((Constants.FieldConstants.FieldX - mDrive.getPose().getX()) /
            mDrive.getPose().getTranslation().getDistance(Constants.FieldConstants.RED_SPEAKER.getTranslation()));
        
      }

      if (mDrive.getPose().getY() < Constants.FieldConstants.BLUE_SPEAKER.getY()) tAngle = tAngle * -1;
        
    return tAngle;
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
}
