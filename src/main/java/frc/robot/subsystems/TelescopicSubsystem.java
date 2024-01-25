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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TelescopicConstants;
import frc.team6014.lib.drivers.SwerveModuleBase;

public class TelescopicSubsystem extends SubsystemBase {

  private TalonFX m_master = new TalonFX(TelescopicConstants.MASTER_MOTOR_ID, "rio");
  private TalonFX m_slave = new TalonFX(TelescopicConstants.SLAVE_MOTOR_ID, "rio");

  private static TelescopicSubsystem m_instance;

  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0, 0, 0, false, false, false);

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, true, false, false, false); // stores output in (next line)
  private double targetOutput = 0; //  output to set in open loop

  private double setpoint = 0;
  private final double sprocketCircumference = 0; // must be in cm!

  private final Trigger brakeModeTrigger;
  private final StartEndCommand brakeModeCommand;

  public enum TelescopicState {
    ZERO,
    OPEN_LOOP,
    CLIMB,
  }

  private TelescopicState telescopicState = TelescopicState.ZERO;

  /** Creates a new TelescopicSubsystem. */
  public TelescopicSubsystem() {
    configureTelescopicMotors();

    brakeModeTrigger = new Trigger(RobotState::isDisabled);
    brakeModeCommand = new StartEndCommand(()->setCoast(), ()->setBreakMode(), this);
    

  }

  private void configureTelescopicMotors() {
    m_master.getConfigurator().apply(new TalonFXConfiguration());
    
    var slot0Configs = new Slot0Configs();
        slot0Configs.kP = TelescopicConstants.TELESCOPIC_CONTROLLER_KP;
        slot0Configs.kI = TelescopicConstants.TELESCOPIC_CONTROLLER_KI;
        slot0Configs.kD = TelescopicConstants.TELESCOPIC_CONTROLLER_KD;
        m_master.getConfigurator().apply(slot0Configs);
    
    var motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicAcceleration = TelescopicConstants.TELESCOPIC_MOTION_ACCEL;
    motionMagicConfigs.MotionMagicCruiseVelocity = TelescopicConstants.TELESCOPIC_MOTION_VEL;
    m_master.getConfigurator().apply(motionMagicConfigs);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    m_master.getConfigurator().apply(motorConfigs);
    m_slave.getConfigurator().apply(motorConfigs);

    m_master.getConfigurator().apply(slot0Configs, TelescopicConstants.TELESCOPIC_MOTION_TIMEOUT);

    // TODO: Configure whether inverted!
    m_slave.setControl(new Follower(m_master.getDeviceID(), false));

    zeroEncoder();

    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch(telescopicState){
      case ZERO:
        stop();
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

    brakeModeTrigger.whileTrue(brakeModeCommand);
    
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

  public void setMotorOutput(){
    m_master.setControl(m_percentOut.withOutput(targetOutput));
  }

  public void openLoop(double percent) {
    if(telescopicState != TelescopicState.OPEN_LOOP){
      telescopicState =  TelescopicState.OPEN_LOOP;
    }
    targetOutput = percent;
  }

  public double getHeight() {
    var masterRot = m_master.getRotorPosition();
    var slaveRot = m_slave.getRotorPosition();
    masterRot.refresh(); 
    slaveRot.refresh();
    double sprocketRotation = (masterRot.getValue() + slaveRot.getValue()) / 2 / TelescopicConstants.TELESCOPIC_GEAR_RATIO;
    return sprocketRotation * sprocketCircumference;
  }

  public void setTelescopicPosition(double target){
    if(telescopicState != TelescopicState.CLIMB){
      telescopicState =  TelescopicState.CLIMB;
    }
    setpoint = target;
  }

  public void setHeight() {
    m_master.setControl(motionMagic.withPosition(setpoint/sprocketCircumference * TelescopicConstants.TELESCOPIC_GEAR_RATIO).withSlot(0));
  }

  public void setTelescopicState(TelescopicState state) {
    telescopicState = state;
  }

  public TelescopicState getTelescopicState() {
    return telescopicState;
  }

  
  public void setBreakMode() {
    m_master.setNeutralMode(NeutralModeValue.Brake);
    //m_master.getConfigurator().apply(motorConfig);
    m_slave.setNeutralMode(NeutralModeValue.Brake);
    //m_slave.getConfigurator().apply(motorConfig);
  } 

  // basically taking them off break mode to trigger cfs 
  // might actually not even use this tbh
  public void setCoast() {
    m_master.setNeutralMode(NeutralModeValue.Coast);
    m_slave.setNeutralMode(NeutralModeValue.Coast);
  }

  public void stop(){
    if(telescopicState != TelescopicState.ZERO){
      telescopicState =  TelescopicState.ZERO;
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
