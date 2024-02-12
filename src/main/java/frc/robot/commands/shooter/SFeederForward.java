// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.FeederState;
import frc.team6014.lib.util.LoggedTunableNumber;

public class SFeederForward extends Command {

  private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private double percentOutput;
  private static final LoggedTunableNumber tunableFeederSpeed = new LoggedTunableNumber("feeder/speed",
      ShooterConstants.FEEDER_OUT);

  /** Creates a new ShooterComand. */
  public SFeederForward(double percentOutput) {
    this.percentOutput = percentOutput;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.setFeederState(FeederState.LET_HIM_COOK);
    shooterSubsystem.setFeederMotorSpeed(tunableFeederSpeed.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooterSubsystem.setFeederMotorSpeed(percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      shooterSubsystem.setFeederState(FeederState.STOP_WAIT_A_SEC);
    shooterSubsystem.setFeederMotorSpeed(0);
    shooterSubsystem.stopFeederMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
