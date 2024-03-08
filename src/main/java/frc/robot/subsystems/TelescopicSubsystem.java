// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicConstants;
import frc.team6014.lib.util.Util;

public class TelescopicSubsystem extends SubsystemBase {

  private TalonFX m_left_motor = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, Constants.RIO_CANBUS);
  private TalonFX m_right_motor = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, Constants.RIO_CANBUS);

  private static TelescopicSubsystem m_instance;

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, false, false, false, false);
  private double targetOutput = 0;

  private double leftOutput = 0;
  private double rightOutput = 0;

  private TelescopicState leftState = TelescopicState.HOLD;
  private TelescopicState rightState = TelescopicState.HOLD;

  /** units: cm */
  private double leftSetpoint = 0;
  private double rightSetpoint = 0;

  NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

  public enum TelescopicState {
    /* neutral - in brake */
    HOLD,
    /* open loop control */
    OPEN_LOOP,
    /* closed-loop motion magic */
    CLIMB,
    /* stop motors */
    STOP,
  }

  /** Creates a new TelescopicSubsystem. */
  public TelescopicSubsystem() {
    configureTelescopicMotors();
  }

  private void configureTelescopicMotors() {
    m_left_motor.getConfigurator().apply(new TalonFXConfiguration());
    m_right_motor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = TelescopicConstants.TELESCOPIC_CONTROLLER_KP;
    configs.Slot0.kI = TelescopicConstants.TELESCOPIC_CONTROLLER_KI;
    configs.Slot0.kD = TelescopicConstants.TELESCOPIC_CONTROLLER_KD;

    configs.Slot0.kS = 0.1; // prev 0.32
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    configs.MotionMagic.MotionMagicAcceleration = TelescopicConstants.TELESCOPIC_MOTION_ACCEL;
    configs.MotionMagic.MotionMagicCruiseVelocity = TelescopicConstants.TELESCOPIC_MOTION_VEL;
    m_left_motor.getConfigurator().apply(configs);
    m_right_motor.getConfigurator().apply(configs);

    zeroEncoders();
    setNeutralMode(NeutralModeValue.Brake);
    m_left_motor.setInverted(true);
  }

  public void maybeHoldCurrentPosition() {
    if (Util.epsilonEquals(m_left_motor.getVelocity().getValueAsDouble(), 0, 0.5)
        && (m_left_motor.getStatorCurrent().getValueAsDouble() > TelescopicConstants.STATOR_CURRENT_LIMIT)) {
      m_left_motor.stopMotor();
      leftState = TelescopicState.HOLD;
    }

    if (Util.epsilonEquals(m_right_motor.getVelocity().getValueAsDouble(), 0, 0.5)
        && (m_right_motor.getStatorCurrent().getValueAsDouble() > TelescopicConstants.STATOR_CURRENT_LIMIT)) {
      m_right_motor.stopMotor();
      rightState = TelescopicState.HOLD;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    maybeHoldCurrentPosition();

    switch (leftState) {
      case HOLD:
        m_left_motor.setControl(new NeutralOut());
        break;
      case OPEN_LOOP:
        setLeftOutput();
        break;
      case STOP:
        stopLeft();
      default:
        stopLeft();
        break;
    }

    switch (rightState) {
      case HOLD:
        m_right_motor.setControl(new NeutralOut());
        break;
      case OPEN_LOOP:
        setRightOutput();
        break;
      case STOP:
        stopRight();
      default:
        stopRight();
        break;
    }

  }

  public double getSetpointLeft() {
    return leftSetpoint;
  }

  public double getSetpointRight() {
    return rightSetpoint;
  }

  public void zeroTelescopicArmLeft() {
    leftSetpoint = 0;
  }

  public void zeroTelescopicArmRight() {
    rightSetpoint = 0;
  }

  public void zeroEncoders() {
    m_left_motor.setPosition(0);
    m_right_motor.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return isAtSetpointLeft() && isAtSetpointRight();
  }

  public boolean isAtSetpointLeft() {
    return Math.abs(leftSetpoint - getLeftHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtSetpointRight() {
    return Math.abs(rightSetpoint - getRightHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZero() {
    return isAtZeroLeft() && isAtZeroRight();
  }

  public boolean isAtZeroLeft() {
    return m_left_motor.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZeroRight() {
    return m_right_motor.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public void setMotorOutput() {
    m_left_motor.setControl(m_percentOut.withOutput(targetOutput));
    m_right_motor.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setLeftOutput() {
    boolean leftExtensionLimit = getLeftHeight() >= TelescopicConstants.MAX_EXTENSION;
    boolean leftRetractionLimit = getLeftHeight() <= TelescopicConstants.MAX_RETRACTION;

    if (leftExtensionLimit && leftOutput > 0) {
      m_left_motor.setControl(new NeutralOut());
    } else if (leftRetractionLimit && leftOutput < 0) {
      m_left_motor.setControl(new NeutralOut());
    } else {
      m_left_motor.setControl(m_percentOut.withOutput(leftOutput));
    }
  }

  public void setRightOutput() {
    boolean rightExtensionLimit = getRightHeight() >= TelescopicConstants.MAX_EXTENSION;
    boolean rightRetractionLimit = getRightHeight() <= TelescopicConstants.MAX_RETRACTION;

    if (rightExtensionLimit && rightOutput > 0) {
      m_right_motor.setControl(new NeutralOut());
    } else if (rightRetractionLimit && rightOutput < 0) {
      m_right_motor.setControl(new NeutralOut());
    } else {
      m_right_motor.setControl(m_percentOut.withOutput(rightOutput));
    }
  }

  public void openLoopLeft(double percent) {
    if (leftState != TelescopicState.OPEN_LOOP) {
      leftState = TelescopicState.OPEN_LOOP;
    }
    leftOutput = percent;
  }

  public void openLoopRight(double percent) {
    if (rightState != TelescopicState.OPEN_LOOP) {
      rightState = TelescopicState.OPEN_LOOP;
    }
    rightOutput = percent;
  }

  /** units: cm */
  public double getLeftHeight() {
    double sprocketRotation = (m_left_motor.getRotorPosition().getValueAsDouble()) // +m_right_motor.getRotorPosition().getValueAsDouble())
        // / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * TelescopicConstants.SPROCKET_CIRCUMFERENCE;
  }

  /** units: cm */
  public double getRightHeight() {
    double sprocketRotation = (m_right_motor.getRotorPosition().getValueAsDouble()) // +m_right_motor.getRotorPosition().getValueAsDouble())
        // / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * TelescopicConstants.SPROCKET_CIRCUMFERENCE;
  }

  public void setTelescopicPosition(double target) {
    setTelescopicPositionLeft(target);
    setTelescopicPositionRight(target);
  }

  public void setTelescopicPositionLeft(double target) {
    if (leftState != TelescopicState.CLIMB) {
      leftState = TelescopicState.CLIMB;
    }
    leftSetpoint = target;
  }

  public void setTelescopicPositionRight(double target) {
    if (rightState != TelescopicState.CLIMB) {
      rightState = TelescopicState.CLIMB;
    }
    rightSetpoint = target;
  }

  public void setTelescopicState(TelescopicState state) {
    setTelescopicStateLeft(state);
    setTelescopicStateRight(state);
  }

  public void setTelescopicStateLeft(TelescopicState state) {
    leftState = state;
  }

  public void setTelescopicStateRight(TelescopicState state) {
    rightState = state;
  }

  public TelescopicState getTelescopicStateLeft() {
    return leftState;
  }

  public TelescopicState getTelescopicStateRight() {
    return rightState;
  }

  public void setNeutralMode() {
    this.kNeutralMode = (kNeutralMode == NeutralModeValue.Brake) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_left_motor.setNeutralMode(this.kNeutralMode);
    m_right_motor.setNeutralMode(this.kNeutralMode);
  }

  public void setNeutralMode(NeutralModeValue value) {
    this.kNeutralMode = value;
    m_left_motor.setNeutralMode(this.kNeutralMode);
    m_right_motor.setNeutralMode(this.kNeutralMode);
  }

  public void stop() {
    stopLeft();
    stopRight();
  }

  public void stopLeft() {
    if (leftState != TelescopicState.STOP) {
      leftState = TelescopicState.STOP;
    }
    m_left_motor.stopMotor();
  }

  public void stopRight() {
    if (rightState != TelescopicState.STOP) {
      rightState = TelescopicState.STOP;
    }
    m_right_motor.stopMotor();
  }

  public void resetEncoder() {
    m_left_motor.setPosition(TelescopicConstants.TELESCOPIC_RESET);
    m_right_motor.setPosition(TelescopicConstants.TELESCOPIC_RESET);
  }

  public static TelescopicSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new TelescopicSubsystem();
    }
    return m_instance;
  }

}
