// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SFeederCommand extends Command {

  private ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private double percentOutput;

  /** Creates a new ShooterComand. */
  public SFeederCommand(double percentOutput) {

    if (percentOutput > 1) {
      this.percentOutput = 1;
    } else if (percentOutput < -1) {
      this.percentOutput = -1;
    } else {
      this.percentOutput = percentOutput;
    }
    addRequirements(shooterSubsystem);
  }

  public SFeederCommand() {
    this(0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getSensorState()) {
      shooterSubsystem.setFeederMotorSpeed(0);
    } else {
      shooterSubsystem.setFeederMotorSpeed(percentOutput);
    }
    shooterSubsystem.setFeederMotorSpeed(percentOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.setFeederMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}