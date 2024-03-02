// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.util.LoggedTunableNumber;

public class TurnToSpeaker extends Command {
  /** Creates a new FieldOrientedTurn. */
  private double m_error, m_lastError, m_output, m_goalAngle, m_currentAngle;
  private LoggedTunableNumber<Number> kP = new LoggedTunableNumber<Number>("AUTO/kP", DriveConstants.kRotControllerP);
  private LoggedTunableNumber<Number> kD = new LoggedTunableNumber<Number>("AUTO/kD", DriveConstants.kRotControllerD);

  private DriveSubsystem mSwerve;
  private boolean atSetpoint = false;

  public TurnToSpeaker(DriveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    mSwerve = swerve;
    m_goalAngle = calcTargetAngle();
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_error = 0;
    m_output = 0;
    // No need to use this, I belive ?
    // m_goalAngle =
    // Conversions.convertAngleByAlliance(DriverStation.getAlliance().get(),
    // m_goalAngle);
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

    m_output = (Constants.isTuning ? kP.get().doubleValue() : DriveConstants.kRotControllerP) * m_error
        + ((Constants.isTuning ? kD.get().doubleValue() : DriveConstants.kRotControllerD) * (m_error - m_lastError));

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

  public double calcTargetAngle() {
    double tAngle = 0, tetha = 0, RobotX = mSwerve.getPose().getX(), RobotY = mSwerve.getPose().getY();

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      tetha = Units.radiansToDegrees(Math.acos(RobotX
          / mSwerve.getPose().getTranslation().getDistance(Constants.FieldConstants.BLUE_SPEAKER.getTranslation())));
      tetha = 180 - tetha;
    } else {
      tetha = Units.radiansToDegrees(
          Math.acos((Constants.FieldConstants.FieldX - RobotX)
              / mSwerve.getPose().getTranslation().getDistance(Constants.FieldConstants.RED_SPEAKER.getTranslation())));
      tetha = -tetha;
    }

    if (RobotY < Constants.FieldConstants.BLUE_SPEAKER.getY())
      tetha = -tetha;

    tAngle = mSwerve.getPose().getRotation().getDegrees() - tetha;

    if (DriverStation.getAlliance().get() == Alliance.Blue)
      tAngle = 360 - tAngle;

    return tAngle;
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
