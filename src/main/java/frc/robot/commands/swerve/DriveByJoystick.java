// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class DriveByJoystick extends Command {

  private final DriveSubsystem mDrive = DriveSubsystem.getInstance();
  private final DoubleSupplier mX;
  private final DoubleSupplier mY;
  private final DoubleSupplier mRotation;
  private final BooleanSupplier mIsLocked;
  private final BooleanSupplier mRush; // higher speed
  private final BooleanSupplier mSteady; // more controlled/slower

  private final SlewRateLimiter mSlewX = new SlewRateLimiter(DriveConstants.driveSlewRateLimitX);
  private final SlewRateLimiter mSlewY = new SlewRateLimiter(DriveConstants.driveSlewRateLimitY);
  private final SlewRateLimiter mSlewRot = new SlewRateLimiter(DriveConstants.driveSlewRateLimitRot);

  private boolean fieldOriented = DriveConstants.isFieldOriented;
  private double driveScalarValue = DriveConstants.drivePowerScalar;

  /** Creates a new DriveByJoystick. */
  public DriveByJoystick(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation, BooleanSupplier isLocked,
      BooleanSupplier rush, BooleanSupplier steady) {
    mX = x; // forward
    mY = y; // strafe
    mRotation = rotation; // rotation
    mIsLocked = isLocked; // is swerve locked
    mRush = rush;
    mSteady = steady;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrive.lockSwerve(mIsLocked.getAsBoolean());

    double scalar = driveScalarValue;
    if (WristSubsystem.getInstance().getState() == WristSubsystem.Position.OPEN) {
      scalar = 0.8;
    } else if (mSteady.getAsBoolean()) {
      scalar = 0.45;
    } else {
      scalar = driveScalarValue;
    }

    double xSpeed = mX.getAsDouble() * DriveConstants.maxSpeed * scalar;
    double ySpeed = mY.getAsDouble() * DriveConstants.maxSpeed * scalar;
    double rotation = mRotation.getAsDouble() * DriveConstants.maxAngularSpeedRadPerSec * scalar;

    if (mIsLocked.getAsBoolean()) {
      mSlewX.reset(0);
      mSlewY.reset(0);
      mSlewRot.reset(0);
    }

    mDrive.swerveDrive(xSpeed, ySpeed, rotation, fieldOriented);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double inputTransform(double input) {
    if (input < 0) {
      return -Math.pow(input, 2);
    } else {
      return Math.pow(input, 2);
    }
  }
}
