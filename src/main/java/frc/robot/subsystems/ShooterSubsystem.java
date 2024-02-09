// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathShared;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.team6014.lib.math.Conversions;

public class ShooterSubsystem extends SubsystemBase {

  /* MOTORS */
  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.SLAVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_feeder = new CANSparkMax(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushed);

  /* SENSORS */
  private DigitalInput m_beamBreaker = new DigitalInput(ShooterConstants.BEAM_BREAK_ID);

  private SparkPIDController m_masterPIDController;
  private SparkPIDController m_slavePIDController;

  private RelativeEncoder m_masterEncoder;
  private RelativeEncoder m_slaveEncoder;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private static ShooterSubsystem m_instance;
  private ShooterState m_shootState;
  private FeederState m_feederState;
  private PowerDistribution m_pdh = new PowerDistribution();

  private boolean isTatmin = Constants.IS_TATMIN;
  double shooter_rpm;
  double feeder_out;
  double shooter_out;

  public enum ShooterState {
    OPEN_LOOP,
    CLOSED,
    AMP,
    SPEAKER,
  }

  public enum FeederState {
    LET_HIM_COOK,
    STOP_WAIT_A_SEC,
  }

  public ShooterSubsystem() {

    feeder_out = 0.0;
    shooter_out = 0.0;
    shooter_rpm = 0.0;

    m_shootState = ShooterState.CLOSED;
    m_feederState = FeederState.STOP_WAIT_A_SEC;

    m_master.restoreFactoryDefaults();
    m_slave.restoreFactoryDefaults();
    m_feeder.restoreFactoryDefaults();

    m_masterPIDController = m_master.getPIDController();
    m_slavePIDController = m_slave.getPIDController();
    m_slaveEncoder = m_slave.getEncoder();
    m_masterEncoder = m_master.getEncoder();

    m_feeder.setIdleMode(ShooterConstants.FEEDER_MODE);

    // PID coefficients
    kP = ShooterConstants.kP;
    kI = ShooterConstants.kI;
    kD = ShooterConstants.kD;
    kIz = ShooterConstants.kIz;
    kFF = ShooterConstants.kFF;
    kMaxOutput = ShooterConstants.kMaxOutput;
    kMinOutput = ShooterConstants.kMinOutput;
    maxRPM = ShooterConstants.maxRPM;

    // set PID coefficients
    m_masterPIDController.setP(kP);
    m_masterPIDController.setI(kI);
    m_masterPIDController.setD(kD);
    m_masterPIDController.setIZone(kIz);
    m_masterPIDController.setFF(kFF);
    m_masterPIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_slavePIDController.setP(kP);
    m_slavePIDController.setI(kI);
    m_slavePIDController.setD(kD);
    m_slavePIDController.setIZone(kIz);
    m_slavePIDController.setFF(kFF);
    m_slavePIDController.setOutputRange(kMinOutput, kMaxOutput);

    m_master.setInverted(ShooterConstants.masterInverted);
    m_slave.setInverted(ShooterConstants.slaveInverted);

    m_slave.follow(m_master, true);

    // m_slave.setInverted(ShooterConstants.slaveInverted);
    // save settings to flash
    m_master.setIdleMode(ShooterConstants.MASTER_MODE);
    m_slave.setIdleMode(ShooterConstants.MASTER_MODE);

    m_master.burnFlash();
    m_slave.burnFlash();
    m_feeder.burnFlash();

  }

  @Override
  public void periodic() {

    if (!isTatmin) {
      if (getSensorState()) {
        m_feederState = FeederState.STOP_WAIT_A_SEC;
        setFeederMotorSpeed(0);
      } else {
        setFeederMotorSpeed(feeder_out);
      }
    }

    switch (m_shootState) {
      case AMP:
        shooter_rpm = ShooterConstants.SPEAKER_SHOOT_RPM;
        break;
      case SPEAKER:
        shooter_rpm = ShooterConstants.AMP_SHOOT_RPM;
        break;
      case OPEN_LOOP:
        setShooterMotorSpeed(shooter_out);
      case CLOSED:
        shooter_rpm = 0;
        break;
      default:
        break;
    }

    if (getFeederState() == FeederState.LET_HIM_COOK) {
      setFeederaMotorSpeed(feeder_out);
    } else {
      setFeederaMotorSpeed(0);
    }

    if (m_shootState != ShooterState.OPEN_LOOP) {
      setShooterTatminMotorsRPM();
    }

    SmartDashboard.putBoolean("Beam Break", m_beamBreaker.get());
    SmartDashboard.putNumber("Shooter RPM", shooter_rpm);
    SmartDashboard.putString("Shooter State", m_shootState.name());

    SmartDashboard.putNumber("Voltage", m_pdh.getVoltage());
    SmartDashboard.putNumber("Total Energy", m_pdh.getTotalEnergy());
    SmartDashboard.putNumber("Total Power", m_pdh.getTotalPower());

  }

  // Setters
  public void setShooterState(ShooterState newState) {
    m_shootState = newState;
  }

  public void setFeederState(FeederState newState) {
    m_feederState = newState;
  }

  public void setFeederMotorSpeed(double percentOutput) {
    this.feeder_out = percentOutput;
  }

  public void setFeederaMotorSpeed(double percentOutput) {
    m_feeder.set(percentOutput);
  }

  public void setShooterOut(double percentOutput) {
    this.shooter_out = percentOutput;
  }

  public void setShooterMotorSpeed(double percentOutput) {
    m_master.set(percentOutput);
    m_slave.set(percentOutput);
  }

  /** unit: rot per minute */
  public void setShooterMotorsRPM(double rpm) {
    m_masterPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    m_slavePIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  private void setShooterTatminMotorsRPM() {
    m_masterPIDController.setReference(shooter_rpm, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.putNumber("Master Encoder RPM", m_masterEncoder.getVelocity());
  }

  // Getters
  public double getMasterMotorSpeed() {
    return m_master.get();
  }

  public double getFeederMotorSpeed() {
    return m_feeder.get();
  }

  /** returns beam break reading */

  public boolean getSensorState() {
    return m_beamBreaker.get();
  }

  public FeederState getFeederState() {
    return m_feederState;
  }

  public ShooterState getShooterState() {
    return m_shootState;
  }

  // Sycentric Othering
  public void stopShMotors() {
    m_master.stopMotor();
    m_slave.stopMotor();
  }

  public void stopFeederMotor() {
    m_feeder.stopMotor();
  }

  public static ShooterSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ShooterSubsystem();
    }
    return m_instance;
  }

}
