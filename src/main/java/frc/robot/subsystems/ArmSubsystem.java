// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(ArmConstants.motorID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreChannel);

  private static ArmSubsystem mInstance;

  private double setpoint = 0; // angle to go by arm
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false, false, false); // stores output in (next line)
  private double targetOutput = 0; //  output to set in open loop
  private ArmControlState armControlState = ArmControlState.OPEN_LOOP;

  private double lastDemandedRotation; // last angle of arm

  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);
  private final PositionTorqueCurrentFOC torqueControl = new PositionTorqueCurrentFOC(0, 0, 0, 1, false, false, false);

  public enum ArmControlState {
    OPEN_LOOP,
    TORQUE_CONTROL,
    MOTION_MAGIC,
  }

  public ArmSubsystem() {
    motorConfig();
    lastDemandedRotation = getArmAngleFalcon();
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem();
    }
    return mInstance;
  }

  private void motorConfig() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    //armMotor.configFactoryDefault();
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = ArmConstants.kP;
    configs.Slot0.kI = ArmConstants.kI;
    configs.Slot0.kD = ArmConstants.kD;

    configs.Slot1.kI = 0;
    configs.Slot1.kP = 0;
    configs.Slot1.kD = 0;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;;
    configs.MotionMagic.MotionMagicAcceleration = 400; // change
    configs.MotionMagic.MotionMagicCruiseVelocity = 120; // change
    configs.MotionMagic.MotionMagicJerk = 800; //  change
    
    //armMotor.setInverted(ArmConstants.motorInverted);
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: CHANGE!
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake; // TODO: CHANGE!
    //armMotor.setNeutralMode(ArmConstants.neutralMode);

    // armMotor.configReverseSoftLimitEnable(true);
    // armMotor.configReverseSoftLimitThreshold(
    //     ArmConstants.gearRatio * ArmConstants.bottomSoftLimit);
    // armMotor.configForwardSoftLimitEnable(true);
    // armMotor.configForwardSoftLimitThreshold(
    //     ArmConstants.gearRatio * ArmConstants.topSoftLimit);

    //armMotor.configVoltageCompSaturation(12);
    //armMotor.enableVoltageCompensation(true);


    //armMotor.configMotionCruiseVelocity(ArmConstants.armCruiseVelocity);
    //armMotor.configMotionAcceleration(ArmConstants.armAcceleration);

    // SupplyCurrentLimitConfiguration m_config = new SupplyCurrentLimitConfiguration();
    // m_config.currentLimit = 15.0; // TODO: Config
    // m_config.enable = true;
    // m_config.triggerThresholdCurrent = 5;
    // m_config.triggerThresholdTime = 2;

    configs.CurrentLimits.StatorCurrentLimit = 300;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 80;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;


    //armMotor.configSupplyCurrentLimit(m_config);
    //armMotor.configClosedloopRamp(ArmConstants.rampRate);
    //armMotor.config_kP(0, ArmConstants.kP);
    //armMotor.config_kI(0, ArmConstants.kI);
    //armMotor.config_kD(0, ArmConstants.kD);

    armMotor.getConfigurator().apply(configs);

    resetFalconEncoder();
    // TODO: CONFIGURE THIS!
    boreEncoder.setDistancePerRotation(ArmConstants.distancePerRotation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Arm Bore Angle", getArmAngleBore()); // will be configured for Bore
    //SmartDashboard.putNumber("Arm Falcon Angle", getArmAngleFalcon());
    //SmartDashboard.putString("Arm State: ", armControlState.toString());
    //SmartDashboard.putNumber("Last Demanded Angle", lastDemandedRotation);


    switch(armControlState){
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setArmAngleMotionMagic();
        break;
      case TORQUE_CONTROL:
        holdPosition();
        break;
      default:
        setArmPercentOutput(0.0);
        break;
    }

    if(!(armControlState == ArmControlState.TORQUE_CONTROL)){
      lastDemandedRotation = getArmAngleFalcon();
    }
    
  }

  // resets falcon encoder to some pre-set zero position
  public void resetFalconEncoder() {
    armMotor.setPosition(ArmConstants.resetAngle * ArmConstants.gearRatio);
  }

  // TODO: Double-check this method
  public boolean isAtSetpointFalcon() {
    return Math.abs(getArmAngleFalcon() - setpoint) < ArmConstants.angleTolerance;
  }

  // TODO: Check difference between rotorPosition and position
  public boolean isAtZeroFalcon() {
    return armMotor.getRotorPosition().getValue() / ArmConstants.gearRatio == ArmConstants.resetAngle;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public double getArmAngleFalcon() {
    return armMotor.getRotorPosition().getValue() *360 / ArmConstants.gearRatio;
  }

  // TODO: Check if we need to multiply by anything here
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition();
  }

  // TODO: decide whether to use getAngle from Falcon or Bore
  public void setArmAngleMotionMagic() {
    // TODO: add max/min angles here!
    //armMotor.configMotionCruiseVelocity(cruiseVel);
    //armMotor.configMotionAcceleration(acc);
    //armMotor.set(ControlMode.MotionMagic, targetAngle * ArmConstants.gearRatio, DemandType.ArbitraryFeedForward,
        //ArmConstants.kF * java.lang.Math.cos(java.lang.Math.toRadians(getArmAngleFalcon())));
    armMotor.setControl(motionMagic.withPosition(setpoint / 360 * ArmConstants.gearRatio));
  }

  public void holdPosition(){
    armMotor.setControl(torqueControl.withPosition(lastDemandedRotation/360 * ArmConstants.gearRatio));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmPercentOutput(double percent) {
    if(armControlState != ArmControlState.OPEN_LOOP){
      armControlState =  ArmControlState.OPEN_LOOP;
    }
    targetOutput = percent;
    lastDemandedRotation = getArmAngleFalcon();
  }

  // sets arm to the respective setpoint
  public void setArmPosition(double target){
    if(armControlState != ArmControlState.MOTION_MAGIC){
      armControlState =  ArmControlState.MOTION_MAGIC;
    }
    setpoint = target;
    lastDemandedRotation = getArmAngleFalcon();
  }

  public void holdArmPosition(){
    if(armControlState != ArmControlState.TORQUE_CONTROL){
      armControlState =  ArmControlState.TORQUE_CONTROL;
    }
  }

  public void updateLastDemandedRotation(double rotation){
    lastDemandedRotation = rotation;
  }

  public void setArmControlState(ArmControlState state){
    armControlState = state;
  }

  public ArmControlState getArmControlState() {
    return armControlState;
  }

  public void setMotorOutput(){
    armMotor.setControl(m_percentOut.withOutput(targetOutput));
  }

}
