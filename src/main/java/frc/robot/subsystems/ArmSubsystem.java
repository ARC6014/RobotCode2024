// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class ArmSubsystem extends SubsystemBase {
  /* MOTORS & ENCODER */
  private final TalonFX armMotor = new TalonFX(ArmConstants.motorID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreChannel);

  private static ArmSubsystem mInstance;
  public Gearbox armGearbox = ArmConstants.gearRatio;

  /**
   * Angle to go by the arm
   * unit: degrees
   */
  private double setpoint = 0;

  // TODO: FOC is false for now
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0);

  /**
   * Output to set in open loop
   * unit: percent
   */
  private double targetOutput = 0;

  private ArmControlState armControlState = ArmControlState.HOLD;

  /** unit: rotations */
  private double lastDemandedRotation;

  /** unit: degrees */
  private double target = 10;

  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0, 0, 1, false, false, false);

  /** unit: rotations */
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public enum ArmControlState {
    OPEN_LOOP,
    MOTION_MAGIC,
    HOLD,
  }

  public ArmSubsystem() {
    motorConfig();
    //lastDemandedRotation = getArmAngleFalcon();

    SmartDashboard.putData("Zero Setpoint", new InstantCommand(() -> zeroSetpoint()));
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem();
    }
    return mInstance;
  }

  private void motorConfig() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration configs = new TalonFXConfiguration();
    // Slot0 is for MotionMagicVoltage
    configs.Slot0.kP = ArmConstants.kP;
    configs.Slot0.kI = ArmConstants.kI;
    configs.Slot0.kD = ArmConstants.kD;
    configs.Slot0.kS = ArmConstants.kS;
    configs.Slot0.kV = ArmConstants.kV;

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    
    configs.MotionMagic.MotionMagicAcceleration = ArmConstants.armAcceleration; // change
    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.armCruiseVelocity; // change
    //configs.CurrentLimits.StatorCurrentLimit = 300;
    //configs.CurrentLimits.StatorCurrentLimitEnable = true;
    //configs.CurrentLimits.SupplyCurrentLimit = 80;
    //configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    armMotor.getConfigurator().apply(configs);

    resetFalconEncoder();
    resetToAbsolute();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Bore Rot", getArmAngleBore());
    SmartDashboard.putNumber("Arm Falcon Rot", getArmAngleFalcon());
    SmartDashboard.putString("Arm State", armControlState.toString());
    SmartDashboard.putNumber("Last Demanded Angle", Conversions.revolutionsToRadians(Math.toDegrees(lastDemandedRotation)));
    SmartDashboard.putBoolean("MotMag Working", armMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled);
    SmartDashboard.putNumber("Falcon Voltage", armMotor.getMotorVoltage().getValueAsDouble());

    switch (armControlState) {
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setArmAngleMotionMagic(target);
        break;
      case HOLD:
        armMotor.setControl(new NeutralOut());
        break;
      default:
        setArmPercentOutput(0.0);
        break;
    }

    //lastDemandedRotation = getArmAngleFalcon();

  }

  // resets falcon encoder to some pre-set zero position
  public void resetFalconEncoder() {
    armMotor.setPosition(0);
  }

  /* Conversions */
  public double drivenToDriver(double revolutions) {
    return revolutions / armGearbox.getRatio();
  }

  public double driverToDriven(double revolutions) {
    return armGearbox.calculate(revolutions);
  }

  // resets falcon encoder so that bore and falcon have the same initial reading
  // theoretically, Falcon position / 119.5 = Bore encoder position + offset at
  // all times
  public void resetToAbsolute() {
    var position = getArmAngleBore() * armGearbox.getRatio();
    armMotor.setPosition(position);
  }

  /** @return true if within angle tolerance */
  public boolean isAtSetpointFalcon() {
    return Math.abs(getArmAngleFalcon() - Conversions.radiansToRevolutions(Math.toRadians(setpoint))) < ArmConstants.angleTolerance;
  }

  public boolean isAtZeroFalcon() {
    return armMotor.getRotorPosition().getValue() / armGearbox.getRatio() < ArmConstants.angleTolerance;
  }

  /** @return setpoint unit: degrees */
  public double getSetpoint() {
    return setpoint;
  }

  public double zeroSetpoint() {
    return setpoint = 0;
  }
  /** unit: revolutions */
  public double getArmAngleFalcon() {
    return armMotor.getRotorPosition().getValueAsDouble() / armGearbox.getRatio();

  }

  /** unit: revolutions */
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition() - boreEncoder.getPositionOffset();
  }

  // TODO: add max/min angles here!
  // TODO: Check rad-angle conversion
  // When using an absolute sensor, such as a CANcoder, the sensor offset must be
  // configured such that a position of 0 represents the arm being held
  // horizontally forward. From there, the RotorToSensor ratio must be configured
  // to the ratio between the absolute sensor and the Talon FX rotor.
  public void setArmAngleMotionMagic(double target) {
    setpoint=target;
    armMotor.setControl(motionMagicVoltage.withPosition(Conversions.radiansToRevolutions(Math.toRadians(setpoint)) * armGearbox.getRatio()));
        //.withFeedForward(ArmConstants.kG * Math.cos(Conversions.revolutionsToRadians(getArmAngleBore()))));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmPercentOutput(double percent) {
    if (armControlState != ArmControlState.OPEN_LOOP) {
      armControlState = ArmControlState.OPEN_LOOP;
    }
    targetOutput = percent;
    lastDemandedRotation = getArmAngleFalcon();
  }


  /** unit: degree */
  public void updateLastDemandedRotation(double rotation) {
    lastDemandedRotation = rotation;
  }

  public void setArmControlState(ArmControlState state) {
    armControlState = state;
  }

  public ArmControlState getArmControlState() {
    return armControlState;
  }

  public void hold() {
    armControlState = ArmControlState.HOLD;
    armMotor.setControl(new NeutralOut());
  }

  public void setMotorOutput() {
    armMotor.setControl(m_percentOut.withOutput(targetOutput));
  }

}
