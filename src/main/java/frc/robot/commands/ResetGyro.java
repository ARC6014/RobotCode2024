// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetGyro extends InstantCommand {

  private DriveSubsystem m_driveSubsystem;
  public ResetGyro(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    m_driveSubsystem = driveSubsystem;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveSubsystem.zeroHeading();

    m_driveSubsystem.poseEstimator = new SwerveDrivePoseEstimator(
      Constants.kinematics,
      m_driveSubsystem.getRotation2d(),
      m_driveSubsystem.getModulePositions(),
      new Pose2d()
    );
    m_driveSubsystem.resetToAbsolute();
  }
}