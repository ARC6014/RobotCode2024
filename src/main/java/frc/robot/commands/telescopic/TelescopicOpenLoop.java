// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.TelescopicSubsystem.TelescopicState;

public class TelescopicOpenLoop extends Command {
  private TelescopicSubsystem mTelescopic = TelescopicSubsystem.getInstance();
  private DoubleSupplier joystick;

  /** Creates a new TelescopicDeneme. */
  public TelescopicOpenLoop(TelescopicSubsystem mTelescopic, DoubleSupplier output) {
    this.mTelescopic = mTelescopic;
    joystick = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTelescopic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTelescopic.setTelescopicState(TelescopicState.OPEN_LOOP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04) {
      mTelescopic.openLoop((joystick.getAsDouble()) / 5);
    } else {
      mTelescopic.setTelescopicState(TelescopicState.HOLD);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTelescopic.setNeutralMode(NeutralModeValue.Brake);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
