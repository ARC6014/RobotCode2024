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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicConstants;

public class TelescopicSubsystem extends SubsystemBase {

  private TalonFX m_master = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, Constants.RIO_CANBUS);
  private TalonFX m_slave = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, Constants.RIO_CANBUS);

  private static TelescopicSubsystem m_instance;

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, false, false, false, false);
  private double targetOutput = 0;

  private double masterOutput = 0;
  private double slaveOutput = 0;

  /** units: cm */
  private double setpoint = 0;

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

  private TelescopicState telescopicState = TelescopicState.HOLD;

  /** Creates a new TelescopicSubsystem. */
  public TelescopicSubsystem() {
    configureTelescopicMotors();
  }

  private void configureTelescopicMotors() {
    m_master.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration configs = new TalonFXConfiguration();
    configs.Slot0.kP = TelescopicConstants.TELESCOPIC_CONTROLLER_KP;
    configs.Slot0.kI = TelescopicConstants.TELESCOPIC_CONTROLLER_KI;
    configs.Slot0.kD = TelescopicConstants.TELESCOPIC_CONTROLLER_KD;

    configs.Slot0.kS = 0.32;
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    configs.MotionMagic.MotionMagicAcceleration = TelescopicConstants.TELESCOPIC_MOTION_ACCEL;
    configs.MotionMagic.MotionMagicCruiseVelocity = TelescopicConstants.TELESCOPIC_MOTION_VEL;
    m_master.getConfigurator().apply(configs);

    m_slave.setControl(new Follower(m_master.getDeviceID(), true));

    zeroEncoder();
    setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (telescopicState) {
      case HOLD:
        m_master.setControl(new NeutralOut());
        break;
      case CLIMB:
        setHeightMotionMagic();
        break;
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case STOP:
        stop();
      default:
        stop();
        break;
    }

  }

  public double getSetpoint() {
    return setpoint;
  }

  public void zeroTelescopicArm() {
    setpoint = 0;
  }

  public void zeroEncoder() {
    m_master.setPosition(0);
    m_slave.setPosition(0);
  }

  public boolean isAtSetpoint() {
    return Math.abs(setpoint - getHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZero() {
    return m_master.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public void setMotorOutput() {
    m_master.setControl(m_percentOut.withOutput(targetOutput));
    m_slave.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void setMasterOutput() {
    m_master.setControl(m_percentOut.withOutput(masterOutput));
  }

  public void setSlaveOutput() {
    m_slave.setControl(m_percentOut.withOutput(slaveOutput));
  }

  public void openLoop(double percent) {
    if (telescopicState != TelescopicState.OPEN_LOOP) {
      telescopicState = TelescopicState.OPEN_LOOP;
    }
    targetOutput = percent;

  }

  public void openLoopMaster(double percent) {
    if (telescopicState != TelescopicState.OPEN_LOOP) {
      telescopicState = TelescopicState.OPEN_LOOP;
    }
    masterOutput = percent;
  }

  public void openLoopSlave(double percent) {
    if (telescopicState != TelescopicState.OPEN_LOOP) {
      telescopicState = TelescopicState.OPEN_LOOP;
    }
    slaveOutput = percent;
  }

  // this is probably wrong double-check
  public double getHeight() {
    double sprocketRotation = (m_master.getRotorPosition().getValueAsDouble()) // +m_slave.getRotorPosition().getValueAsDouble())
                                                                               // / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * TelescopicConstants.SPROCKET_CIRCUMFERENCE;
  }

  public void setTelescopicPosition(double target) {
    if (telescopicState != TelescopicState.CLIMB) {
      telescopicState = TelescopicState.CLIMB;
    }
    setpoint = target;
  }

  // double check
  private void setHeightMotionMagic() {
    m_master.setControl(
        motionMagic.withPosition(
            setpoint / TelescopicConstants.SPROCKET_CIRCUMFERENCE * TelescopicConstants.TELESCOPIC_GEAR_RATIO));
  }

  public void setTelescopicState(TelescopicState state) {
    telescopicState = state;
  }

  public TelescopicState getTelescopicState() {
    return telescopicState;
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
    if (telescopicState != TelescopicState.STOP) {
      telescopicState = TelescopicState.STOP;
    }
    m_master.stopMotor();
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
