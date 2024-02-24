// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SetIdleModeInvert extends Command {

  private boolean isFinished = false;
  private final IntakeSubsystem m_intakeSubsystem = IntakeSubsystem.getInstance();
  private final WristSubsystem m_wristSubsystem = WristSubsystem.getInstance();
  // private final TelescopicSubsystem m_telescopicSubsystem = TelescopicSubsystem.getInstance();

  public SetIdleModeInvert() {
  }

  @Override
  public void initialize() {
    m_intakeSubsystem.setNeutralMode();
    m_wristSubsystem.setNeutralMode();
    // m_telescopicSubsystem.setNeutralMode();
    isFinished = true;

  }

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
    return isFinished;
  }
}
