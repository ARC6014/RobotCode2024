// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final WPI_TalonFX armMotor = new WPI_TalonFX(ArmConstants.motorID, Constants.CANIVORE_CANBUS);
  
  private static ArmSubsystem mInstance;
  private double setpoint = 0;
  private ArmState armState;

  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreChannel);

  public enum ArmState {
    ZERO,
    INTAKE,
    AMP,
    SPEAKER,
  }

  public ArmSubsystem() {
    motorConfig();
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem();
    }
    return mInstance;
  }

  private void motorConfig() {
    armMotor.configFactoryDefault();

    armMotor.setInverted(ArmConstants.motorInverted);
    armMotor.setNeutralMode(ArmConstants.neutralMode);

    armMotor.configReverseSoftLimitEnable(true);
    armMotor.configReverseSoftLimitThreshold(
        ArmConstants.gearRatio * ArmConstants.bottomSoftLimit);
    armMotor.configForwardSoftLimitEnable(true);
    armMotor.configForwardSoftLimitThreshold(
        ArmConstants.gearRatio * ArmConstants.topSoftLimit);

    armMotor.configVoltageCompSaturation(12);
    armMotor.enableVoltageCompensation(true);

    armMotor.configMotionCruiseVelocity(ArmConstants.armCruiseVelocity);
    armMotor.configMotionAcceleration(ArmConstants.armAcceleration);

    SupplyCurrentLimitConfiguration m_config = new SupplyCurrentLimitConfiguration();
    m_config.currentLimit = 15.0; // TODO: Config
    m_config.enable = true;
    m_config.triggerThresholdCurrent = 5;
    m_config.triggerThresholdTime = 2;

    armMotor.configSupplyCurrentLimit(m_config);
    armMotor.configClosedloopRamp(ArmConstants.rampRate);
    armMotor.config_kP(0, ArmConstants.kP);
    armMotor.config_kI(0, ArmConstants.kI);
    armMotor.config_kD(0, ArmConstants.kD);

    resetFalconEncoder();
    // TODO: CONFIGURE THIS!
    boreEncoder.setDistancePerRotation(ArmConstants.distancePerRotation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Arm Bore Angle", getArmAngleBore()); // will be configured for Bore
    SmartDashboard.putNumber("Arm Falcon Angle", getArmAngleFalcon());
  }

  // resets falcon encoder to some pre-set zero position
  public void resetFalconEncoder() {
    armMotor.setSelectedSensorPosition(ArmConstants.resetAngle * ArmConstants.gearRatio);
  }

  // TODO: Double-check this method
  public boolean isAtSetpointFalcon() {
    return Math.abs(getArmAngleFalcon() - setpoint) < ArmConstants.angleTolerance;
  }

  public boolean isAtZeroFalcon() {
    return armMotor.getSelectedSensorPosition() / ArmConstants.gearRatio == ArmConstants.resetAngle;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void setSetpoint(double target) {
    setpoint = target;
  }

  public double getArmAngleFalcon() {
    return armMotor.getSelectedSensorPosition() / ArmConstants.gearRatio;
  }

  // TODO: Check if we need to multiply by anything here
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition();
  }

  // TODO: decide whether to use getAngle from Falcon or Bore
  public void setArmAngleMotionMagic(double targetAngle, double cruiseVel, double acc) {
    armMotor.configMotionCruiseVelocity(cruiseVel);
    armMotor.configMotionAcceleration(acc);
    armMotor.set(ControlMode.MotionMagic, targetAngle * ArmConstants.gearRatio, DemandType.ArbitraryFeedForward,
        ArmConstants.kF * java.lang.Math.cos(java.lang.Math.toRadians(getArmAngleFalcon())));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmPercentOutput(double percent) {
    armMotor.set(ControlMode.PercentOutput, percent);
  }

}
