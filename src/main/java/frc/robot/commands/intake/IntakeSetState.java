// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Intake control command (open loop for angle only).
 * 
 * This command is awkward for a few reasons:
 * - This is a single subsystem wrapping two independent states,
 * of two different mechanisms.
 * - If we use addRequirements(), it forces Commands setting those
 * two independent states to cancel each other.
 * 
 * The solution here is a single command that changes both states at once.
 * We'll do state changes via Trigger classes; we'll put them into the arguments of this command,
 * getting the changed state from the button 
 * and the unchanged one from IntakeSubsystem::getIntake{Running, Position}.
 * An alternative solution is to break the IntakeSubsystem into two independent subsystems
 * and have commands for each.
 */
public class IntakeSetState extends Command {
  /** Creates a new IntakeClosedLoop. */
  private IntakeSubsystem mIntakeSubsystem;
  private DoubleSupplier mOpenLoopOutput;

  private IntakeSubsystem.Position mPosition;
  private IntakeSubsystem.Running mRunning;

  public IntakeSetState(
    IntakeSubsystem intakeSubsystem, 
    DoubleSupplier openLoopOutput, 
    IntakeSubsystem.Position position, 
    IntakeSubsystem.Running running
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    mIntakeSubsystem = intakeSubsystem;
    mOpenLoopOutput = openLoopOutput;
    mPosition = position;
    mRunning = running;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mIntakeSubsystem.setPositionState(mPosition);
    mIntakeSubsystem.setRunningState(mRunning);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mIntakeSubsystem.getIntakePosition() == IntakeSubsystem.Position.OPENLOOP) {
      mIntakeSubsystem.setAngleOpenLoop(mOpenLoopOutput.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mIntakeSubsystem.isAtPositionSetpoint() && mIntakeSubsystem.isAtVelocitySetpoint();
  }
}
