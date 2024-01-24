// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.telescopic;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TelescopicConstants;

public class TelescopicSubsystem extends SubsystemBase {

  private TalonFX m_master = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, "rio");
  private TalonFX m_slave = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, "rio");

  private static TelescopicSubsystem m_instance;


  private MotorOutputConfigs motorConfig = new MotorOutputConfigs();

  private double setpoint;
  private double sensorUnitPerClimb = (TelescopicConstants.TELESCOPIC_GEAR_RATIO*2048) / (1.75*2.54*Math.PI);

  public enum TelescopicArmState {
    ZERO,
    CLIMB,
  }

  public void setSetpoint(double number) {
    setpoint = number;
  }

  public double getSetpoint() {
    return setpoint;
  }

  public void zeroTelescopicArm() {
    setpoint = 0;
  }

  public boolean isAtSetpoint() {
    return Math.abs(setpoint - getHeight()) < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public boolean isAtZero() {
    return m_master.getRotorPosition().getValue() < TelescopicConstants.TELESCOPIC_TOLERANCE;
  }

  public void openLoop(double output) {
    final DutyCycleOut m_request = new DutyCycleOut(0);
    m_master.setControl(m_request.withOutput(output));
    m_slave.setControl(m_request.withOutput(output));

    //m_master.set(ControlMode.PercentOutput, output);
    //m_slave.set(ControlMode.PercentOutput, output);
  }

  public double getHeight() {
    double sensorUnit = m_master.getRotorPosition().getValue();
    return sensorUnit / sensorUnitPerClimb;
  }

  private void runMotor() {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    m_master.setControl(m_request.withPosition(distanceToRotation(setpoint)));
    //m_master.set(ControlMode.MotionMagic, Conversions.radiansToSteps(distanceToRotation(setpoint) / 180 * Math.PI, ElevatorConstants.ELEVATOR_GEAR_RATIO));

  }

  private double distanceToRotation(double distance) {
    return distance / TelescopicConstants.DISTANCE_PER_UNIT_DEGREE;
  }

  /** Creates a new TelescopicSubsystem. */
  public TelescopicSubsystem() {
    configureTelescopicMotors();
    motorConfig.NeutralMode = NeutralModeValue.Brake;
  }

  public void setBreakMode() {
    //m_master.setNeutralMode(NeutralModeValue.Brake);
    m_master.getConfigurator().apply(motorConfig);
    //m_slave.setNeutralMode(NeutralModeValue.Brake);
    m_slave.getConfigurator().apply(motorConfig);
  } 

  private void configureTelescopicMotors() {
    m_master.getConfigurator().apply(new TalonFXConfiguration());
    
    TalonFXConfiguration config = new TalonFXConfiguration();
    var slot0Configs = new Slot0Configs();
        slot0Configs.kP = TelescopicConstants.TELESCOPIC_CONTROLLER_KP;
        slot0Configs.kI = TelescopicConstants.TELESCOPIC_CONTROLLER_KI;
        slot0Configs.kD = TelescopicConstants.TELESCOPIC_CONTROLLER_KD;
        m_master.getConfigurator().apply(slot0Configs);
    
    var motionMagiConfigs = new MotionMagicConfigs();
    motionMagiConfigs.MotionMagicAcceleration = TelescopicConstants.TELESCOPIC_MOTION_ACCEL;
    motionMagiConfigs.MotionMagicCruiseVelocity = TelescopicConstants.TELESCOPIC_MOTION_VEL;
    m_master.getConfigurator().apply(motionMagiConfigs);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    m_master.getConfigurator().apply(motorConfigs);
    m_slave.getConfigurator().apply(motorConfigs);

    m_master.getConfigurator().apply(slot0Configs, TelescopicConstants.TELESCOPIC_MOTION_TIMEOUT);
  }

  public void resetEncoder() {
    m_master.setPosition(TelescopicConstants.TELESCOPIC_RESET);
    m_slave.setPosition(TelescopicConstants.TELESCOPIC_RESET);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //runMotor();
  }

  public static TelescopicSubsystem getInstance() {
    if (m_instance == null) {
      m_instance = new TelescopicSubsystem();
    }
    return m_instance;
  }

  public void TelescopicStateSet(TelescopicArmState level) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'TelescopicStateSet'");
  }

}
