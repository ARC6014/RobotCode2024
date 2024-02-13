// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.math.Conversions;

public class FieldOrientedTurn extends Command {
  /** Creates a new FieldOrientedTurn. */
  private double m_error, m_lastError, m_output, m_goalAngle, m_currentAngle;

  private DriveSubsystem mSwerve;
  private boolean atSetpoint = false;

  public FieldOrientedTurn(DriveSubsystem swerve, double goalAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = swerve;
    m_goalAngle = goalAngle;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_error = 0;
    m_output = 0;
    m_goalAngle = Conversions.convertAngleByAlliance(DriverStation.getAlliance().get(), m_goalAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentAngle = mSwerve.getPose().getRotation().getDegrees();
    m_error = m_goalAngle - m_currentAngle;
    if (m_error > 180) {
      m_goalAngle = m_goalAngle - 360;
      m_error = m_goalAngle - m_currentAngle;
    } else if (m_error < -180) {
      m_goalAngle = m_goalAngle + 360;
      m_error = m_goalAngle - m_currentAngle;
    }

    m_output = DriveConstants.kRotControllerP * m_error
        + (DriveConstants.kRotControllerD * (m_error - m_lastError));

    if (m_error >= -DriveConstants.kRotControllerTolerance
        && m_error <= DriveConstants.kRotControllerTolerance) {
      m_output = 0;
      atSetpoint = true;
    }

    m_lastError = m_error;
    m_output = Conversions.clamp(
        m_output, -DriveConstants.kRotControllerMaxVel, DriveConstants.kRotControllerMaxVel);
    mSwerve.swerveDrive(0, 0, m_output, true);

    SmartDashboard.putBoolean("Field Turn AtSetpoint", atSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    atSetpoint = false;
    mSwerve.swerveDrive(0, 0, 0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoint;
  }
}
