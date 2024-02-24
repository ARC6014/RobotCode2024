// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.team6014.lib.math.Conversions;

public class ArmOnlyBore extends SubsystemBase {
  /** Creates a new ArmOnlyBore. */
  private static ArmOnlyBore mInstance;
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.BORE_ID);

  public ArmOnlyBore() {
    boreEncoder.setPositionOffset(ArmConstants.POSITION_OFFSET);
  }

  public static ArmOnlyBore getInstance() {
    if (mInstance == null) {
      mInstance = new ArmOnlyBore();
    }
    return mInstance;
  }

  /** unit: revolutions */
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition() - boreEncoder.getPositionOffset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Bore Angle", Conversions.revolutionsToDegrees(getArmAngleBore()));
  }
}
