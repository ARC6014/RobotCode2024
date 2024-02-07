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

  /** Creates a new ShooterComand. */
  public ShooterCommand() {
    percentOutput = 0;
    addRequirements(mShooterSubsystem);
  }

  public ShooterCommand withOpenLoop(double percentOutput) {
    mShooterSubsystem.setShooterState(ShooterState.OPEN_LOOP);
    this.percentOutput = percentOutput;
    return this;
  }

  public ShooterCommand withShooterState(ShooterState level) {
    mShooterSubsystem.setShooterState(level);
    return this;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {

    if (mShooterSubsystem.getShooterState() == ShooterState.OPEN_LOOP) {
      mShooterSubsystem.setShooterMotorSpeed(percentOutput);
    }

    switch (mShooterSubsystem.getShooterState()) {

      case OPEN_LOOP:
        mShooterSubsystem.setShooterMotorSpeed(percentOutput);
        break;

      case AMP:
        mShooterSubsystem.setShooterMotorsRPM(ShooterConstants.AMP_SHOOT_RPM);
        break;

      case SPEAKER:
        mShooterSubsystem.setShooterMotorsRPM(ShooterConstants.SPEAKER_SHOOT_RPM);
        break;

      default:
        mShooterSubsystem.setShooterMotorsRPM(0);
        break;
    }

  }

  @Override
  public void end(boolean interrupted) {
    mShooterSubsystem.setShooterState(ShooterState.CLOSED);
    mShooterSubsystem.stopShMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
