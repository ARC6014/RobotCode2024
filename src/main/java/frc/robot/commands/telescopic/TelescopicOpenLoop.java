// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.telescopic;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TelescopicSubsystem;
import frc.robot.subsystems.TelescopicSubsystem.TelescopicState;

public class TelescopicOpenLoop extends Command {
  private TelescopicSubsystem mTelescopic = TelescopicSubsystem.getInstance();
  private DoubleSupplier joystick1;
  private DoubleSupplier joystick2;

  private SlewRateLimiter limiterMaster = new SlewRateLimiter(0.05);
  private SlewRateLimiter limiterSlave = new SlewRateLimiter(0.05);

  /** Creates a new TelescopicDeneme. */
  public TelescopicOpenLoop(TelescopicSubsystem mTelescopic, DoubleSupplier masterOutput, DoubleSupplier slaveOutput) {
    this.mTelescopic = mTelescopic;
    joystick1 = masterOutput;
    joystick2 = slaveOutput;
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
    double masterOut = MathUtil.applyDeadband(MathUtil.clamp(joystick1.getAsDouble(), 0, 0.33), 0.04);
    double slaveOut = MathUtil.applyDeadband(MathUtil.clamp(joystick2.getAsDouble(), 0, 0.33), 0.04);

    mTelescopic.openLoopMaster(limiterMaster.calculate(masterOut));
    mTelescopic.openLoopSlave(limiterSlave.calculate(slaveOut));

    if (masterOut == 0) {
      mTelescopic.setTelescopicStateMaster(TelescopicState.HOLD);
    } 
    if (slaveOut == 0) {
      mTelescopic.setTelescopicStateSlave(TelescopicState.HOLD);
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
