package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import frc.robot.Constants.LEDConstants;


public class CANdleLed {
    private static CANdleLed instance = new CANdleLed();
    CANdle candle;
    CANdleConfiguration config;
    RainbowAnimation rainbowAnim;
    StrobeAnimation strobeAnim;
    StrobeAnimation pinkStrobe;
    StrobeAnimation purpleStrobe;
    StrobeAnimation yellowStrobe;
    LarsonAnimation larsonAnim;
    FireAnimation FIREEE;
    ColorFlowAnimation colorFLow;
    SingleFadeAnimation orangeFade;


    public CANdleLed(){
        candle = new CANdle(LEDConstants.CANDLE_ID);
        configSettings();

    }

    public static CANdleLed getInstance(){
        return instance;
    }

    public void setRainbow(){
        candle.animate(rainbowAnim);
    }

    public void setStrobe(){
        candle.animate(strobeAnim);
    }


    public void setPinkStrobe(){
        candle.animate(pinkStrobe);
    }

    public void setPurpleStrobe(){
        candle.animate(purpleStrobe);
    }

    public void setYellowStrobe(){
        candle.animate(yellowStrobe);
    }

    public void setGreenStrobe(){
        candle.animate(strobeAnim);
    }

    public void setPewPew(){
        candle.animate(larsonAnim);
    }

    public void offAnim(){
        candle.clearAnimation(0);
    }

    public void setRed(){
        candle.setLEDs(255, 0, 0);
    }

    public void setGreen(){
        candle.setLEDs(0, 255, 0);
    }

    public void setBlue(){
        candle.setLEDs(0, 0, 255);
    }

    public void setYellow(){
        offAnim();
        candle.setLEDs(255, 100, 0);
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public void setOrange(){
         
        offAnim();
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
        candle.setLEDs(255, 25, 0);
        
    }

    public void setOrangeFade(){
        
        candle.animate(orangeFade);
    }



    public void setOff(){
        offAnim();
        candle.setLEDs(0, 0, 0);
    }



    public void setPurple(){
        offAnim();
        candle.setLEDs(255, 0, 50);
        config.brightnessScalar = 1;
        candle.configAllSettings(config);
    }

    public ErrorCode getLastError(){
        return candle.getLastError();
    }
    private void configSettings(){
        config = new CANdleConfiguration();
        candle.configFactoryDefault();
        config.stripType = LEDStripType.RGB;

        config.brightnessScalar = .5;
        candle.setLEDs(255, 255, 255);
        candle.clearAnimation(0);
        rainbowAnim = new RainbowAnimation(1, 0.85, LEDConstants.BUFFER_LENGTH);
        strobeAnim = new StrobeAnimation(0, 255, 0, 0, 0, LEDConstants.BUFFER_LENGTH);
        pinkStrobe = new StrobeAnimation(255, 0, 255, 0, 0, LEDConstants.BUFFER_LENGTH);
        yellowStrobe = new StrobeAnimation(255, 100, 0, 0 , 0, LEDConstants.BUFFER_LENGTH);
        purpleStrobe = new StrobeAnimation(255, 0, 50, 0, 0, LEDConstants.BUFFER_LENGTH, 300);
        FIREEE = new FireAnimation(1, 1, LEDConstants.BUFFER_LENGTH, 0.5, 0.5);
        colorFLow = new ColorFlowAnimation(255, 25, 0, 0, 0.5, LEDConstants.BUFFER_LENGTH, Direction.Forward, 0);
        larsonAnim = new LarsonAnimation(255, 25, 0, 0, 0.5, LEDConstants.BUFFER_LENGTH, BounceMode.Back, 25);
        orangeFade = new SingleFadeAnimation(255, 25, 0, 0, 0.5, LEDConstants.BUFFER_LENGTH);
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.vBatOutputMode = VBatOutputMode.Off;
        candle.configAllSettings(config);
    }
}