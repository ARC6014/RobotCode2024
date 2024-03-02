// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

public class AllignWithLL extends Command {
  /** Creates a new AllignWithLL. */
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  // private LoggedTunableNumber x_distance = new LoggedTunableNumber("x
  // distance", 1.5);

  private double ySpeed = 0, xSpeed = 0, tethaSpeed = 0;

  private final ProfiledPIDController x_pid = new ProfiledPIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD, DriveConstants.transPIDconstraints);
  private final ProfiledPIDController y_pid = new ProfiledPIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD, DriveConstants.transPIDconstraints);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(DriveConstants.anglekP,
      DriveConstants.anglekI,
      DriveConstants.anglekD, DriveConstants.rotPIDconstraints);

  private Pose2d targetPose = new Pose2d(1.5, 6, new Rotation2d(Math.PI)); // blue speakerish

  public AllignWithLL() {
    addRequirements(mDrive);

    x_pid.setTolerance(0.015);
    y_pid.setTolerance(0.015);
    m_thetaController.setTolerance(Math.toRadians(1));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_pid.reset(mDrive.getPose().getX());
    y_pid.reset(mDrive.getPose().getY());
    m_thetaController.reset(mDrive.getPose().getRotation().getRadians());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*
     * Yagsl
     * PathConstraints constraints = new PathConstraints(
     * swerveDrive.getMaximumVelocity(), 4.0,
     * swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720));
     * 
     * // Since AutoBuilder is configured, we can use it to build pathfinding
     * commands
     * return AutoBuilder.pathfindToPose(
     * pose,
     * constraints,
     * 0.0, // Goal end velocity in meters/sec
     * 0.0 // Rotation delay distance in meters. This is how far the robot should
     * travel before attempting to rotate.
     */

    Pose2d currentPose = mDrive.getPose();

    double xSpeed = x_pid.calculate(currentPose.getX(), targetPose.getX());
    double ySpeed = y_pid.calculate(currentPose.getY(), targetPose.getY());
    double tethaSpeed = m_thetaController.calculate(currentPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians());

    mDrive.swerveDrive(xSpeed, ySpeed, tethaSpeed, true); // not sure if it should be fieldRelative or not

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("tethaSpeed", tethaSpeed);
    SmartDashboard.putBoolean("İs Aligned", x_pid.atGoal());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrive.swerveDrive(0, 0, 0, true); // not sure if it should be fieldRelative or not
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x_pid.atGoal() && y_pid.atGoal() && m_thetaController.atGoal();
  }

}
