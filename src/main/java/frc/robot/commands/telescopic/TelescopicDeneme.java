// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.telescopic.TelescopicSubsystem;

public class TelescopicDeneme extends Command {


  private final TelescopicSubsystem m_arm = TelescopicSubsystem.getInstance();
  private final DoubleSupplier joystick;

  /** Creates a new TelescopicDeneme. */
  public TelescopicDeneme(DoubleSupplier output) {
    joystick = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      m_arm.openLoop((joystick.getAsDouble())/5);
    }
    else {
      m_arm.setBreakMode();
    }

    SmartDashboard.putNumber("Arm Height", m_arm.getHeight());
    SmartDashboard.putBoolean("Is at zero?", m_arm.isAtZero());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
