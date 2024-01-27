// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants.DriveConstants;


public class AllignWithLL extends Command {
  /** Creates a new AllignWithLL. */
  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  private final LimelightSubsystem mLL = LimelightSubsystem.getInstance();
  
  private double currID = 0, xSpeed = 0, tethaSpeed = 0;

  private double tagIDtoAllign;

  private final ProfiledPIDController x_pid = new ProfiledPIDController(DriveConstants.drivekP, DriveConstants.drivekI, 
    DriveConstants.drivekD, DriveConstants.transPIDconstraints);
  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(DriveConstants.anglekP, DriveConstants.anglekI,
  DriveConstants.anglekD, DriveConstants.rotPIDconstraints);
  
  public AllignWithLL(double tagID) {
    tagIDtoAllign = tagID; 
    addRequirements(mDrive); 
    addRequirements(mLL);

    x_pid.setTolerance(0.015);
    m_thetaController.setTolerance(Math.toRadians(1));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_pid.reset(mLL.getCamPose3d_target().getX());
    m_thetaController.reset(mLL.getCamPose3d_target().getRotation().getZ());
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currID = mLL.getID();

    if (currID == tagIDtoAllign){
      xSpeed = x_pid.calculate(mLL.getCamPose3d_target().getX(), 0);
      tethaSpeed = m_thetaController.calculate(mLL.getCamPose3d_target().getRotation().getZ(), 0);
    } else {
      System.out.println("April tag is not the target");
    }

    mDrive.swerveDrive(xSpeed, 0, tethaSpeed, false); //not sure if it should be fieldRelative or not
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return x_pid.atGoal() && m_thetaController.atGoal();
  }

}
