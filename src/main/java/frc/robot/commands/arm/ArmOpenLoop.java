// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmOpenLoop extends Command {
  /** Creates a new ArmOpenLoop. */
  private ArmSubsystem mArm;
  private final DoubleSupplier joystick;

  public ArmOpenLoop(ArmSubsystem arm, DoubleSupplier output) {
    mArm = arm;
    joystick = output;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.updateLastDemandedRotation(mArm.getArmAngleFalcon());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getAsDouble() >= 0.04 || joystick.getAsDouble() <= -0.04){
      mArm.setArmPercentOutput(joystick.getAsDouble() / 5);
    }
    else {
      mArm.setArmPercentOutput(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mArm.setArmPercentOutput(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
