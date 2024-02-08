// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class TatminShooter extends Command {

  private ShooterSubsystem mShooterSubsystem;
  private ShooterState state;

  public TatminShooter(ShooterSubsystem shooterSubsystem, ShooterState state) {
    this.mShooterSubsystem = shooterSubsystem;
    addRequirements(mShooterSubsystem);
    this.state = state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooterSubsystem.setShooterState(state);
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
    return false;
  }
}
