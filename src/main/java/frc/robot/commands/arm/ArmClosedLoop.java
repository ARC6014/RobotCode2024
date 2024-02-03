// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmClosedLoop extends Command {
  /** Creates a new ArmClosedLoop. */
  private ArmSubsystem mArm;
  private double setpoint;
  private double finishPoint;
  private boolean finishable, stop;

  public ArmClosedLoop(ArmSubsystem arm, double setpoint, double finishPoint, Boolean stop) {
    mArm = arm;
    this.setpoint = setpoint;
    finishable = true;
    this.finishPoint = finishPoint;
    this.stop = stop;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mArm.setArmAngleMotionMagic(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stop) {
      mArm.setArmPercentOutput(0.0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Format get arm angle and finishpoint tolerance
    if (finishable
        && mArm.getArmAngleFalcon() > (finishPoint - 1.0)
        && mArm.getArmAngleFalcon() < (finishPoint + 1.0)) {
      return true;
    }

    return false;
  }
}
