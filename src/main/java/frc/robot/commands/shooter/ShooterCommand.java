// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ShooterCommand extends Command {

  private ShooterSubsystem mShooterSubsystem = ShooterSubsystem.getInstance();
  private double percentOutput;
  private ShooterState state;

  /** Creates a new ShooterComand. */
  public ShooterCommand() {
    percentOutput = 0;
    addRequirements(mShooterSubsystem);
  }

  public ShooterCommand withOpenLoop(double percentOutput) {
    this.state = ShooterState.OPEN_LOOP;
    this.percentOutput = percentOutput;
    return this;
  }

  public ShooterCommand withShooterState(ShooterState state) {
    this.state = state;
    return this;
  }

  @Override
  public void initialize() {
    mShooterSubsystem.setShooterState(state);

    if (state == ShooterState.OPEN_LOOP) {
      mShooterSubsystem.setShooterOut(percentOutput);
    }

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    mShooterSubsystem.setShooterState(ShooterState.CLOSED);
    mShooterSubsystem.setShooterOut(0);
    mShooterSubsystem.stopShMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
