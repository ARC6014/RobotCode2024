// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class ShooterComand extends Command {

  public ShooterState level;
  private ShooterSubsystem mShooterSubsystem;
  
  /** Creates a new ShooterComand. */
  public ShooterComand(ShooterState level) {
    this.level = level;
    addRequirements(ShooterSubsystem.getInstance());
    mShooterSubsystem.setShooterState(level);

  }

  public ShooterComand(ShooterState level, double openLoopOut) {
    this.level = level;
    addRequirements(ShooterSubsystem.getInstance());
    mShooterSubsystem.setShooterState(level);

    mShooterSubsystem.setShooterMotorSpeed(openLoopOut);
  }
  

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    if (mShooterSubsystem.getShooterState() == ShooterState.AMP) {
      mShooterSubsystem.setShooterMotorsRPM(ShooterConstants.AMP_SHOOT_RPM);
    } 
    else if (mShooterSubsystem.getShooterState() == ShooterState.SPEAKER){
      mShooterSubsystem.setShooterMotorsRPM(ShooterConstants.SPEAKER_SHOOT_RPM);
    }
    else { //hareket etmesin pls
      mShooterSubsystem.setShooterMotorsRPM(0);
    }
    
  }

  @Override
  public void end(boolean interrupted) {
    level = ShooterState.CLOSED;
    mShooterSubsystem.setShooterState(level);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
