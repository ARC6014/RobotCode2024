// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.MASTER_MOTOR_ID, MotorType.kBrushless);
  private DigitalInput m_beamBreaker = new DigitalInput(ShooterConstants.BEAM_BREAK_ID);
  private SparkPIDController m_masterPIDController;
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.SLAVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_feeder = new CANSparkMax(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushed);
  private SparkPIDController m_slavePIDController;

  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  private static ShooterSubsystem m_instance;
  private ShooterState m_shootState;
  private FeederState m_feederState;

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

    m_shootState = ShooterState.CLOSED;
    m_feederState = FeederState.LET_HIM_COOK;

    m_master.restoreFactoryDefaults();
    m_slave.restoreFactoryDefaults();
    m_feeder.restoreFactoryDefaults();

    m_masterPIDController = m_master.getPIDController();
    m_slavePIDController = m_slave.getPIDController();

    m_feeder.setIdleMode(ShooterConstants.FEEDER_MODE);
    m_master.setIdleMode(ShooterConstants.MASTER_MODE);
    m_slave.setIdleMode(ShooterConstants.MASTER_MODE);

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
    // save settings to flash
    m_master.burnFlash();
    m_slave.burnFlash();
  }

  @Override
  public void periodic() {
    if (getSensorState()) {
      m_feederState = FeederState.STOP_WAIT_A_SEC;
    } else {
      m_feederState = FeederState.LET_HIM_COOK;
    }
  }

  // Setters

  public void setShooterState(ShooterState newState) {
    m_shootState = newState;
  }

  public void setFeederState(FeederState newState) {
    m_feederState = newState;
  }

  public void setFeederMotorSpeed(double percentOutput) {
    m_feeder.set(percentOutput);
  }

  public void setShooterMotorSpeed(double percentOutput) {
    m_master.set(percentOutput);
    m_slave.set(percentOutput);
  }

  public void setShooterMotorsRPM(double rpm) {
    m_masterPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    m_slavePIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  public void invertMotors(boolean isInverted) {
    m_master.setInverted(isInverted);
    m_slave.setInverted(isInverted);
  }

  public void stopShMotors() {
    m_master.stopMotor();
    m_slave.stopMotor();
  }

  // Getters
  public double getMasterMotorSpeed() {
    return m_master.get();
  }

  public double getFeederMotorSpeed() {
    return m_feeder.get();
  }

  public boolean getSensorState() {
    return m_beamBreaker.get();
  }

  public FeederState getFeederState() {
    return m_feederState;
  }

  public ShooterState getShooterState() {
    return m_shootState;
  }

  public static ShooterSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new ShooterSubsystem();
    }
    return m_instance;
  }

}
