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
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

public class AddressableLEDSubsystem extends SubsystemBase implements Loggable {
  /** Creates a new AddressableLED. */
  private AddressableLEDSubsystem m_instance;
  private AddressableLED mLED = new AddressableLED(LEDConstants.PWM_PORT);
  private AddressableLEDBuffer mLEDBuffer = new AddressableLEDBuffer(LEDConstants.BUFFER_LENGTH);

  public AddressableLEDSubsystem() {
    mLED.setLength(mLEDBuffer.getLength());
    mLED.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (RobotState.isDisabled()) {
      setLEDColor(Color.kPink);
    }

    setLEDTailColor(Color.kBlack);
  }

  public void setLEDColor(Color color) {
    for (int i = 0; i < mLEDBuffer.getLength() - 1; i++) {
      mLEDBuffer.setLED(i, color);
    }
    mLED.setData(mLEDBuffer);

  }

  public void setLEDTailColor(Color color) {
    for (int i = mLEDBuffer.getLength() - 1; i >= mLEDBuffer.getLength() - 5; i--) {
      mLEDBuffer.setLED(i, color);
    }
    mLED.setData(mLEDBuffer);

  }

  public void turnOffLED() {
    setLEDColor(Color.kBlack);
    mLED.setData(mLEDBuffer);
  }

  @Log(name = "LED Length", width = 1, height = 1, rowIndex = 0, columnIndex = 0)
  public int getLength() {
    return mLEDBuffer.getLength();
  }

  public AddressableLEDSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new AddressableLEDSubsystem();
    }
    return m_instance;
  }
}
