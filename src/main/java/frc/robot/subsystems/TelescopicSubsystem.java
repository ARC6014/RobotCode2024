// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.TelescopicConstants;
import frc.team6014.lib.drivers.SwerveModuleBase;

public class TelescopicSubsystem extends SubsystemBase {

  private TalonFX m_master = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, Constants.RIO_CANBUS);
  private TalonFX m_slave = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, Constants.RIO_CANBUS);

  private static TelescopicSubsystem m_instance;

  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, false, false, false, false);
  private double targetOutput = 0; 

  /** units: cm */
  private double setpoint = 0;

  /** units: cm */
  private final double sprocketCircumference = 0;


  public enum TelescopicState {
    ZERO,
    HOLD,
    OPEN_LOOP,
    CLIMB,
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

    configs.Slot0.kS = 0.32; // Add 0.25 V output to overcome static friction
    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    configs.MotionMagic.MotionMagicAcceleration = TelescopicConstants.TELESCOPIC_MOTION_ACCEL; 
    configs.MotionMagic.MotionMagicCruiseVelocity = TelescopicConstants.TELESCOPIC_MOTION_VEL; 
    m_master.getConfigurator().apply(configs);

    // TODO: Configure whether inverted!
    m_slave.setControl(new Follower(m_master.getDeviceID(), false));

    zeroEncoder();
    setBreakMode();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (telescopicState) {
      case ZERO:
        stop();
        break;
      case HOLD:
        m_master.setControl(new NeutralOut());
        break;
      case CLIMB:
        setHeight();
        break;
      case OPEN_LOOP:
        setMotorOutput();
        break;
      default:
        stop();
        break;
    }

    SmartDashboard.putNumber("Telescopic Height", getHeight());
    SmartDashboard.putString("State", getTelescopicState().toString());
    SmartDashboard.putBoolean("Is at zero?", isAtZero());

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
  }

  public void openLoop(double percent) {
    if (telescopicState != TelescopicState.OPEN_LOOP) {
      telescopicState = TelescopicState.OPEN_LOOP;
    }
    targetOutput = percent;
  }

  public double getHeight() {
    double sprocketRotation = (m_master.getRotorPosition().getValueAsDouble() + m_slave.getRotorPosition().getValueAsDouble()) / 2
        / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * sprocketCircumference;
  }

  public void setTelescopicPosition(double target) {
    if (telescopicState != TelescopicState.CLIMB) {
      telescopicState = TelescopicState.CLIMB;
    }
    setpoint = target;
  }

  public void hold() {
    telescopicState = TelescopicState.HOLD;
    m_master.setControl(new NeutralOut());
  }

  public void setHeight() {
    m_master.setControl(motionMagic
        .withPosition(setpoint / sprocketCircumference * TelescopicConstants.TELESCOPIC_GEAR_RATIO));
  }

  public void setTelescopicState(TelescopicState state) {
    telescopicState = state;
  }

  public TelescopicState getTelescopicState() {
    return telescopicState;
  }

  public void setBreakMode() {
    m_master.setNeutralMode(NeutralModeValue.Brake);
    m_slave.setNeutralMode(NeutralModeValue.Brake);
  }

  // might actually not even use this tbh
  public void setCoast() {
    m_master.setNeutralMode(NeutralModeValue.Coast);
    m_slave.setNeutralMode(NeutralModeValue.Coast);
  }

  public void stop() {
    if (telescopicState != TelescopicState.ZERO) {
      telescopicState = TelescopicState.ZERO;
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
