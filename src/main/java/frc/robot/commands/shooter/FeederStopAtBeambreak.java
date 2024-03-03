// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class FeederStopAtBeambreak extends Command {
  /** Creates a new ShooterStopAtBeambreak. */
  private ShooterSubsystem mShooter = ShooterSubsystem.getInstance();

  public FeederStopAtBeambreak() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * Will return TRUE if there is no object, so feeder should spin,
     * return FALSE for there is object, so feeder stop
     */
    return !mShooter.getSensorState();
  }
}
