// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmControlState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmStateSet extends Command {
  private ArmSubsystem mArm;
  private ArmControlState mTarget;

  public ArmStateSet(ArmSubsystem arm, ArmControlState targetState) {
    mArm = arm;
    mTarget = targetState;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mArm.setArmControlState(mTarget);
    mArm.updateLastDemandedRotation(mArm.getArmAngleFalcon());
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return mArm.isAtSetpointFalcon();
  }
}
