// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.team6014.lib.decorators.Assign;
import frc.team6014.lib.decorators.Assign.Prog;

public class RotateToSpeaker extends Command {
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(7,
      DriveConstants.anglekI,
      DriveConstants.anglekD, DriveConstants.rotPIDconstraints);

  @Assign(users = { Prog.Carabelli, Prog.CAN, Prog.Xerem }, message = "Angle FeedForward Ekleyek bier ara")
  private final SimpleMotorFeedforward m_Feedforward = new SimpleMotorFeedforward(1, 0.1, 0);
  private double tethaSpeed = 0;

  // in Degrees
  private double targetAngle;

  /** Creates a new RotateToSpeaker. */
  public RotateToSpeaker() {
    m_thetaController.setTolerance(Math.toRadians(3));
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
    SmartDashboard.putNumber("LL Target Angle", targetAngle);
    SmartDashboard.putBoolean("LL Rotate SP", m_thetaController.atGoal());
    SmartDashboard.putNumber("LL T S", (tethaSpeed));
  }

  public double calcTargetAngle() {
    double tAngle = 0;
    double tetha = 0;
    double RobotX = mDrive.getPose().getX();
    double RobotY = mDrive.getPose().getY();
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      tetha = Units.radiansToDegrees(Math.acos(RobotX
          / mDrive.getPose().getTranslation().getDistance(Constants.FieldConstants.BLUE_SPEAKER.getTranslation())));
      tAngle = 180 - tetha;
    } else {
      tetha = Units.radiansToDegrees(
          Math.acos((Constants.FieldConstants.FieldX - RobotX)
              / mDrive.getPose().getTranslation().getDistance(Constants.FieldConstants.RED_SPEAKER.getTranslation())));
      tAngle = -tetha;
    }

    if (RobotY < Constants.FieldConstants.BLUE_SPEAKER.getY())
      tAngle = -tAngle;

    if (tAngle < 0) {
      tAngle += 360;
    }
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
