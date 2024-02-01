// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.WristSubsystem.Position;

public class WristOpenLoop extends Command {
  /** Creates a new WristOpenLoop. */
  private WristSubsystem mWrist;
  private DoubleSupplier mOutputSupplier;

  public WristOpenLoop(WristSubsystem wrist, DoubleSupplier output) {
    // Use addRequirements() here to declare subsystem dependencies.
    mWrist = wrist;
    mOutputSupplier = output;
    addRequirements(mWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mWrist.setState(Position.OPENLOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = mOutputSupplier.getAsDouble();
    if (output > 0.04 || output < -0.04) {
      mWrist.setOpenLoop(output / 5);
    } else {
      mWrist.setOpenLoop(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mWrist.setOpenLoop(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
