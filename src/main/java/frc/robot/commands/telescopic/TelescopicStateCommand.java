// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;
import frc.robot.subsystems.TelescopicSubsystem.TelescopicState;
import frc.robot.subsystems.TelescopicSubsystem;

public class TelescopicStateCommand extends Command {

  private TelescopicSubsystem mTelescopic = TelescopicSubsystem.getInstance();
  private TelescopicState state;
  private double target;

  public TelescopicStateCommand() {
    target = 0;
    addRequirements(mTelescopic);
  }

  public TelescopicStateCommand withArbitrarySet(double target) {
    this.state = TelescopicState.CLIMB;
    this.target = target;
    return this;
  }

  public TelescopicStateCommand withShooterState(TelescopicState state) {
    this.state = state;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTelescopic.setTelescopicState(state);

    if (state == TelescopicState.CLIMB) {
      mTelescopic.setTelescopicPosition(target);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTelescopic.setNeutralMode(NeutralModeValue.Brake);
    mTelescopic.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
