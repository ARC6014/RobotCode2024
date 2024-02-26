// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.util.LoggedTunableNumber;

public class ShooterSubsystem extends SubsystemBase {

  /* MOTORS */
  private CANSparkMax m_master = new CANSparkMax(ShooterConstants.MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slave = new CANSparkMax(ShooterConstants.SLAVE_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_feeder = new CANSparkMax(ShooterConstants.FEEDER_MOTOR_ID, MotorType.kBrushed);

  /* SENSORS */
  private DigitalInput m_beamBreaker;

  private SparkPIDController m_masterPIDController;
  private SparkPIDController m_slavePIDController;

  private double kMaxOutput, kMinOutput;

  private static ShooterSubsystem m_instance;
  private ShooterState m_shootState;
  private FeederState m_feederState;

  double feeder_out;
  double shooter_slave_out;
  double shooter_master_out;

  LoggedTunableNumber isShooterVoltage = new LoggedTunableNumber("Shooter Is Voltage Mode",
      ShooterConstants.IS_VOLTAGE_MODE);

  public enum ShooterState {
    OPEN_LOOP,
    CLOSED,
    AMP,
    SPEAKER_LONG,
    SPEAKER_SHORT
  }

  public enum FeederState {
    LET_HIM_COOK,
    UPSI,
    STOP_WAIT_A_SEC,
    OPEN,
    // feeder from intake
    INTAKECEPTION
  }

  public ShooterSubsystem() {

    m_beamBreaker = new DigitalInput(ShooterConstants.BEAM_BREAK_ID);

    feeder_out = 0.0;
    shooter_master_out = 0.0;
    shooter_slave_out = 0.0;

    m_shootState = ShooterState.CLOSED;
    m_feederState = FeederState.STOP_WAIT_A_SEC;

    m_master.restoreFactoryDefaults();
    m_slave.restoreFactoryDefaults();
    m_feeder.restoreFactoryDefaults();

    m_master.setSmartCurrentLimit(65);
    m_slave.setSmartCurrentLimit(65);
    // m_feeder.setSmartCurrentLimit(25);
    m_master.setOpenLoopRampRate(0.2); // reduce this if it slows down shooter
    m_slave.setOpenLoopRampRate(0.2); // reduce this if it slows down shooter

    m_masterPIDController = m_master.getPIDController();
    m_slavePIDController = m_slave.getPIDController();

    m_feeder.setIdleMode(ShooterConstants.FEEDER_MODE);

    // PID coefficients
    kMaxOutput = ShooterConstants.kMaxOutput;
    kMinOutput = ShooterConstants.kMinOutput;

    m_masterPIDController.setOutputRange(kMinOutput, kMaxOutput);
    m_slavePIDController.setOutputRange(kMinOutput, kMaxOutput);

    // save settings to flash
    m_master.setIdleMode(ShooterConstants.MASTER_MODE);
    m_slave.setIdleMode(ShooterConstants.MASTER_MODE);

    // slave should turn in opposite with the master
    m_master.setInverted(true);
    // m_slave.follow(m_master);
    m_slave.setInverted(true);

    m_master.burnFlash();
    m_slave.burnFlash();
    m_feeder.burnFlash();

  }

  public boolean isShooterStopped() {
    double rpm = m_master.getEncoder().getVelocity();
    return rpm < 10;
  }

  @Override
  public void periodic() {

    // if (!getSensorState() && isShooterStopped()) {
    // m_feederState = FeederState.STOP_WAIT_A_SEC;
    // }

    SmartDashboard.putNumber("SH-Master RPM",
        m_master.getEncoder().getVelocity());
    SmartDashboard.putNumber("SH-Slave RPM", m_slave.getEncoder().getVelocity());
    SmartDashboard.putNumber("SH-Master-Current", m_master.getOutputCurrent());
    SmartDashboard.putNumber("SH-Slave-Current", m_slave.getOutputCurrent());
    SmartDashboard.putBoolean("Beam Break Reading", getSensorState());
    SmartDashboard.putNumber("PDH V", RobotContainer.mPDH.getVoltage());
    if (isShooterVoltage.getBoolean()) {

      return;
    }

    switch (m_shootState) {
      case AMP:
        setAmpOut(Constants.ShooterConstants.AMP_VOLTAGE);
        break;
      case SPEAKER_LONG:
        setShooterOut(Constants.ShooterConstants.SPEAKER_LONG_VOLTAGE);
        break;
      case SPEAKER_SHORT:
        setShooterOut(Constants.ShooterConstants.SPEAKER_SHORT_VOLTAGE);
        break;
      case OPEN_LOOP:
        break;
      case CLOSED:
      default:
        setShooterOut(0);
        break;
    }

    switch (m_feederState) {
      case LET_HIM_COOK:
        setFeederOUT(Constants.ShooterConstants.FEEDER_OUT);
        break;
      case UPSI:
        setFeederOUT(Constants.ShooterConstants.FEEDER_REVERSE);
        break;
      case INTAKECEPTION:
        setFeederOUT(Constants.ShooterConstants.FEEDER_FROM_INTAKE);
        break;
      case OPEN:
        break;
      case STOP_WAIT_A_SEC:
      default:
        setFeederOUT(0);
        break;
    }

    setShooterMotorSpeed();
    setFeederMotorSpeed(feeder_out);

  }

  // Setters
  public void setShooterState(ShooterState newState) {
    if (isShooterVoltage.getBoolean()) {
      switch (m_shootState) {
        case AMP:
          setShooterVoltage(Constants.ShooterConstants.AMP_VOLTAGE);
          break;
        case SPEAKER_LONG:
          setShooterVoltage(Constants.ShooterConstants.SPEAKER_LONG_VOLTAGE);
          break;
        case SPEAKER_SHORT:
          setShooterVoltage(Constants.ShooterConstants.SPEAKER_SHORT_VOLTAGE);
          break;
        case OPEN_LOOP:
          break;
        case CLOSED:
        default:
          setShooterVoltage(0);
          break;
      }
    }
    m_shootState = newState;
  }

  public void setFeederState(FeederState newState) {
    m_feederState = newState;
  }

  public void setFeederMotorSpeed(double percentOutput) {
    m_feeder.set(percentOutput);

  }

  public void setFeederOUT(double percentOutput) {
    var optimalOut = Conversions.getSmartVoltage(percentOutput, RobotContainer.mPDH.getVoltage());
    this.feeder_out = optimalOut;
  }

  // TODO: Add scalar here to adjust slave output to master
  public void setShooterOut(double voltage) {
    var optimalOutMaster = Conversions.getSmartVoltage(voltage, RobotContainer.mPDH.getVoltage());
    var optimalOutSlave = Conversions.getSmartVoltage(voltage * ShooterConstants.SLAVE_FUDGE_FACTOR,
        RobotContainer.mPDH.getVoltage());

    this.shooter_master_out = optimalOutMaster;
    this.shooter_slave_out = optimalOutSlave;
  }

  // TODO: Add scalar here to adjust slave output to master
  public void setAmpOut(double percentOutput) {
    var optimalOutMaster = Conversions.getSmartVoltage(percentOutput, RobotContainer.mPDH.getVoltage());
    this.shooter_master_out = optimalOutMaster;
    this.shooter_slave_out = 0;
  }

  public void setShooterMotorSpeed() {
    SmartDashboard.putNumber("Shooter Master Out", shooter_master_out);
    SmartDashboard.putNumber("Shooter Slave Out", shooter_slave_out);
    m_master.set(shooter_master_out);
    m_slave.set(shooter_slave_out);
  }

  public void setShooterVoltage(double voltage) {
    m_master.setVoltage(voltage);
    m_slave.setVoltage(voltage * ShooterConstants.SLAVE_FUDGE_FACTOR);
  }

  /** unit: rot per minute */
  public void setShooterMotorsRPM(double rpm) {
    m_masterPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    m_slavePIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  // Getters

  public double getMasterMotorSpeed() {
    return m_master.get();
  }

  public double getFeederMotorSpeed() {
    return m_feeder.get();
  }

  /** returns true for no object */
  public boolean getSensorState() {
    return m_beamBreaker.get();
  }

  public FeederState getFeederState() {
    return m_feederState;
  }

  public ShooterState getShooterState() {
    return m_shootState;
  }

  public double getShooterPercentTargetMaster() {
    return shooter_master_out;
  }

  public double getShooterPercentTargetSlave() {
    return shooter_slave_out;
  }

  public double getFeederPercentTarget() {
    return feeder_out;
  }

  public void stopShMotors() {
    // setShooterMotorSpeed(0);
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
