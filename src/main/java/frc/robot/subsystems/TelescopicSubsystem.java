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

  private TalonFX m_master = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, Constants.RIO_CANBUS);
  private TalonFX m_slave = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, Constants.RIO_CANBUS);

  private static TelescopicSubsystem m_instance;

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, false, false, false, false);
  private double targetOutput = 0;

  private double masterOutput = 0;
  private double slaveOutput = 0;
  private TelescopicState masterState = TelescopicState.HOLD;
  private TelescopicState slaveState = TelescopicState.HOLD;

  /** units: cm */
  private double masterSetpoint = 0;
  private double slaveSetpoint = 0;

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
    m_master.getConfigurator().apply(new TalonFXConfiguration());
    m_slave.getConfigurator().apply(new TalonFXConfiguration());

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
    m_master.getConfigurator().apply(configs);
    m_slave.getConfigurator().apply(configs);

    zeroEncoders();
    setNeutralMode(NeutralModeValue.Brake);
  }

  public void maybeHoldCurrentPosition() {
    if (Util.epsilonEquals(m_master.getVelocity().getValueAsDouble(), 0, 0.5)
        && (m_master.getStatorCurrent().getValueAsDouble() > TelescopicConstants.STATOR_CURRENT_LIMIT)) {
      m_master.stopMotor();
      masterState = TelescopicState.HOLD;
    }

    if (Util.epsilonEquals(m_slave.getVelocity().getValueAsDouble(), 0, 0.5)
        && (m_slave.getStatorCurrent().getValueAsDouble() > TelescopicConstants.STATOR_CURRENT_LIMIT)) {
      m_slave.stopMotor();
      slaveState = TelescopicState.HOLD;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    maybeHoldCurrentPosition();

    switch (masterState) {
      case HOLD:
        m_master.setControl(new NeutralOut());
        break;
      case CLIMB:
        setHeightMotionMagicMaster();
        break;
      case OPEN_LOOP:
        setMasterOutput();
        break;
      case STOP:
        stopMaster();
      default:
        stopMaster();
        break;
    }

    switch (slaveState) {
      case HOLD:
        m_slave.setControl(new NeutralOut());
        break;
      case CLIMB:
        setHeightMotionMagicSlave();
        break;
      case OPEN_LOOP:
        setSlaveOutput();
        break;
      case STOP:
        stopSlave();
      default:
        stopSlave();
        break;
    }

    if (Constants.DEVELOPER_LOGGING) {
      if (this.getCurrentCommand() != null) {
        SmartDashboard.putString("Scheduled", this.getCurrentCommand().toString());
      } else {
        SmartDashboard.putString("Scheduled", "No command");
      }

      SmartDashboard.putNumber("Teles Master Out", masterOutput);
      SmartDashboard.putNumber("Tele Slave Out", slaveOutput);
      SmartDashboard.putNumber("Tele Master Height", getMasterHeight());
      SmartDashboard.putNumber("Tele Slave Height", getSlaveHeight());
    }

  }

  public double getSetpointMaster() {
    return masterSetpoint;
  }

  public double getSetpointSlave() {
    return slaveSetpoint;
  }

  public void zeroTelescopicArmMaster() {
    masterSetpoint = 0;
  }

  public void zeroTelescopicArmSlave() {
    slaveSetpoint = 0;
  }

  public void zeroEncoders() {
    m_master.setPosition(0);
    m_slave.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return isAtSetpointMaster() && isAtSetpointSlave();
  }

  public boolean isAtSetpointMaster() {
    return Math.abs(masterSetpoint - getMasterHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtSetpointSlave() {
    return Math.abs(slaveSetpoint - getSlaveHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZero() {
    return isAtZeroMaster() && isAtZeroSlave();
  }

  public boolean isAtZeroMaster() {
    return m_master.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZeroSlave() {
    return m_slave.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public void setMotorOutput() {
    m_master.setControl(m_percentOut.withOutput(targetOutput));
    m_slave.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setMasterOutput() {
    boolean masterExtensionLimit = getMasterHeight() >= TelescopicConstants.MAX_EXTENSION;
    boolean masterRetractionLimit = getMasterHeight() <= TelescopicConstants.MAX_RETRACTION;

    if (masterExtensionLimit && masterOutput > 0) {
      m_master.setControl(new NeutralOut());
    } else if (masterRetractionLimit && masterOutput < 0) {
      m_master.setControl(new NeutralOut());
    } else {
      m_master.setControl(m_percentOut.withOutput(masterOutput));
    }
  }

  public void setSlaveOutput() {
    boolean slaveExtensionLimit = getSlaveHeight() >= TelescopicConstants.MAX_EXTENSION;
    boolean slaveRetractionLimit = getSlaveHeight() <= TelescopicConstants.MAX_RETRACTION;

    if (slaveExtensionLimit && slaveOutput > 0) {
      m_slave.setControl(new NeutralOut());
    } else if (slaveRetractionLimit && slaveOutput < 0) {
      m_slave.setControl(new NeutralOut());
    } else {
      m_slave.setControl(m_percentOut.withOutput(slaveOutput));
    }
  }

  public void openLoopMaster(double percent) {
    if (masterState != TelescopicState.OPEN_LOOP) {
      masterState = TelescopicState.OPEN_LOOP;
    }
    masterOutput = percent;
  }

  public void openLoopSlave(double percent) {
    if (slaveState != TelescopicState.OPEN_LOOP) {
      slaveState = TelescopicState.OPEN_LOOP;
    }
    slaveOutput = percent;
  }

  /** units: cm */
  public double getMasterHeight() {
    double sprocketRotation = (m_master.getRotorPosition().getValueAsDouble()) // +m_slave.getRotorPosition().getValueAsDouble())
                                                                               // / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * TelescopicConstants.SPROCKET_CIRCUMFERENCE;
  }

  /** units: cm */
  public double getSlaveHeight() {
    double sprocketRotation = (m_slave.getRotorPosition().getValueAsDouble()) // +m_slave.getRotorPosition().getValueAsDouble())
                                                                              // / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * TelescopicConstants.SPROCKET_CIRCUMFERENCE;
  }

  public void setTelescopicPosition(double target) {
    setTelescopicPositionMaster(target);
    setTelescopicPositionSlave(target);
  }

  public void setTelescopicPositionMaster(double target) {
    if (masterState != TelescopicState.CLIMB) {
      masterState = TelescopicState.CLIMB;
    }
    masterSetpoint = target;
  }

  public void setTelescopicPositionSlave(double target) {
    if (slaveState != TelescopicState.CLIMB) {
      slaveState = TelescopicState.CLIMB;
    }
    slaveSetpoint = target;
  }

  // double check
  private void setHeightMotionMagicMaster() {
    m_master.setControl(
        motionMagic.withPosition(
            masterSetpoint / TelescopicConstants.SPROCKET_CIRCUMFERENCE * TelescopicConstants.TELESCOPIC_GEAR_RATIO));
  }

  // double check
  private void setHeightMotionMagicSlave() {
    m_master.setControl(
        motionMagic.withPosition(
            slaveSetpoint / TelescopicConstants.SPROCKET_CIRCUMFERENCE * TelescopicConstants.TELESCOPIC_GEAR_RATIO));
  }

  public void setTelescopicState(TelescopicState state) {
    setTelescopicStateMaster(state);
    setTelescopicStateSlave(state);
  }

  public void setTelescopicStateMaster(TelescopicState state) {
    masterState = state;
  }

  public void setTelescopicStateSlave(TelescopicState state) {
    slaveState = state;
  }

  public TelescopicState getTelescopicStateMaster() {
    return masterState;
  }

  public TelescopicState getTelescopicStateSlave() {
    return slaveState;
  }

  public void setNeutralMode() {
    this.kNeutralMode = (kNeutralMode == NeutralModeValue.Brake) ? NeutralModeValue.Coast : NeutralModeValue.Brake;
    m_master.setNeutralMode(this.kNeutralMode);
    m_slave.setNeutralMode(this.kNeutralMode);
  }

  public void setNeutralMode(NeutralModeValue value) {
    this.kNeutralMode = value;
    m_master.setNeutralMode(this.kNeutralMode);
    m_slave.setNeutralMode(this.kNeutralMode);
  }

  public void stop() {
    stopMaster();
    stopSlave();
  }

  public void stopMaster() {
    if (masterState != TelescopicState.STOP) {
      masterState = TelescopicState.STOP;
    }
    m_master.stopMotor();
  }

  public void stopSlave() {
    if (slaveState != TelescopicState.STOP) {
      slaveState = TelescopicState.STOP;
    }
    m_slave.stopMotor();
  }

  public void resetEncoder() {
    m_master.setPosition(TelescopicConstants.TELESCOPIC_RESET);
    m_slave.setPosition(TelescopicConstants.TELESCOPIC_RESET);
  }

  public static TelescopicSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new TelescopicSubsystem();
    }
    return m_instance;
  }

}
