// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FeederState;

public class FeederCommand extends Command {

  private ShooterSubsystem mFeederSubsystem = ShooterSubsystem.getInstance();
  private double percentOutput;
  private FeederState state;

  /** Creates a new FeederComand. */
  public FeederCommand() {
    percentOutput = 0;
  }

  public FeederCommand withArbitraryOut(double percentOutput) {
    this.state = FeederState.OPEN;
    this.percentOutput = percentOutput;
    return this;
  }

  public FeederCommand withFeederState(FeederState state) {
    System.out.println("Feeder state yaptik amk");
    this.state = state;
    return this;
  }

  @Override
  public void initialize() {
    mFeederSubsystem.setFeederState(state);

    if (state == FeederState.OPEN) {
      mFeederSubsystem.setFeederOUT(percentOutput);
    }

  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    mFeederSubsystem.setFeederState(FeederState.STOP_WAIT_A_SEC);
    mFeederSubsystem.stopShMotors();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
