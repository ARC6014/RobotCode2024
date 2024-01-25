// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class CANdleLed extends SubsystemBase {
    /** Creates a new CANdle. */
    private static CANdleLed m_instance = new CANdleLed();

    private final CANdle m_candle = new CANdle(Constants.LEDConstants.CANdleID, "rio");

    private Animation m_toAnimate = null;

    public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll,
        Amplified,
        Default,
        Speaker,
        AMP,
    }

    private AnimationTypes m_currentAnimation;

    public CANdleLed() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(config, 100);
        changeAnimation(AnimationTypes.SetAll);

    }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;

        switch (toChange) {
            case ColorFlow:
                m_toAnimate = new ColorFlowAnimation(128, 20, 70, 0, 0.7, Constants.LEDConstants.BUFFER_LENGTH,
                        Direction.Forward);
                break;
            case Fire:
                m_toAnimate = new FireAnimation(0.5, 0.7, Constants.LEDConstants.BUFFER_LENGTH, 0.7, 0.5);
                break;
            case Larson:
                m_toAnimate = new LarsonAnimation(0, 255, 46, 0, 1, Constants.LEDConstants.BUFFER_LENGTH,
                        BounceMode.Front, 3);
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, Constants.LEDConstants.BUFFER_LENGTH);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.4, Constants.LEDConstants.BUFFER_LENGTH);
                break;
            case SingleFade:
                m_toAnimate = new SingleFadeAnimation(50, 2, 200, 0, 0.5, Constants.LEDConstants.BUFFER_LENGTH);
                break;
            case Strobe:
                m_toAnimate = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, Constants.LEDConstants.BUFFER_LENGTH);
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, Constants.LEDConstants.BUFFER_LENGTH,
                        TwinklePercent.Percent6);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, Constants.LEDConstants.BUFFER_LENGTH,
                        TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
            case Default: // orange
                m_toAnimate = new ColorFlowAnimation(255, 140, 0, 0, 0.7, LEDConstants.BUFFER_LENGTH,
                        Direction.Forward);
            case Speaker:
                m_toAnimate = new ColorFlowAnimation(34, 139, 34, 0, 0.7, LEDConstants.BUFFER_LENGTH,
                        Direction.Forward);
            case AMP:
                m_toAnimate = new ColorFlowAnimation(255, 0, 0, 0, 0.7, LEDConstants.BUFFER_LENGTH, Direction.Forward);
            case Amplified:
                m_toAnimate = new RainbowAnimation(1, 0.5, LEDConstants.BUFFER_LENGTH);

        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_toAnimate == null) {
            m_candle.setLEDs(255, 192, 203);
        } else {
            m_candle.animate(m_toAnimate);
        }
    }

    public AnimationTypes getCurrentAnimation() {
        return m_currentAnimation;
    }

    public static CANdleLed getInstance() {
        return m_instance;
    }
}
