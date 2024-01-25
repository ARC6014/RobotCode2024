// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.TelescopicSubsystem.TelescopicState;

public class TelescopicDeneme extends Command {


  private final TelescopicSubsystem telescopic = TelescopicSubsystem.getInstance();
  private final DoubleSupplier joystick;

  /** Creates a new TelescopicDeneme. */
  public TelescopicDeneme(DoubleSupplier output) {
    joystick = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(telescopic);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      telescopic.openLoop((joystick.getAsDouble())/5);
    }
    else {
      telescopic.setTelescopicState(TelescopicState.ZERO);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    telescopic.setBreakMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
