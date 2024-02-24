// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLED. */
  private static AddressableLED mLED;
  private static AddressableLEDBuffer mLEDBuffer;

  public AddressableLEDSubsystem(int pwmPort) {
    mLED = new AddressableLED(pwmPort);
    mLEDBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_LENGTH);
    mLED.setLength(mLEDBuffer.getLength());
    mLED.start();
    mLED.setData(mLEDBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotState.isDisabled()) {
      setLEDColor(Color.kPink);
    } else if(RobotState.isEnabled()) {
      setLEDColor(Color.kAqua);
    } else {
      turnOffLED();
    }

    // setLEDTailColor(Color.kBlack);
  }


  public void setLEDColor(Color color) {
    for (int i = 0; i < mLEDBuffer.getLength() - 1; i++) {
      mLEDBuffer.setLED(i, color);
    }
    mLED.setData(mLEDBuffer);

  }

  /* 
  public void setLEDTailColor(Color color) {
        for (int i = mLEDBuffer.getLength() -1; i >= mLEDBuffer.getLength() - 5; i--) {
        mLEDBuffer.setLED(i, color);
        }
        mLED.setData(mLEDBuffer);

  } */

  public void turnOffLED() {
    setLEDColor(Color.kBlack);
    mLED.setData(mLEDBuffer);
  }
}
