// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.telescopic.TelescopicSubsystem;
import frc.robot.subsystems.telescopic.TelescopicSubsystem.TelescopicArmState;

public class TelescopicStateSet extends Command {


  private TelescopicArmState level;
  private TelescopicSubsystem m_arm;

  public TelescopicStateSet(TelescopicArmState level, TelescopicSubsystem m_armm) {
    this.level = level;
    m_armm = m_arm;
  }


  /** Creates a new TelescopicStateSet. */
  public TelescopicStateSet() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.TelescopicStateSet(level);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
