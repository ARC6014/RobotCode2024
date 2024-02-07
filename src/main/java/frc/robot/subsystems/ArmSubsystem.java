// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.arm.ArmStateSet;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class ArmSubsystem extends SubsystemBase {
  /* MOTORS & ENCODER */
  private final TalonFX armMotor = new TalonFX(ArmConstants.motorID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreChannel);

  private static ArmSubsystem mInstance;
  public Gearbox armGearbox = ArmConstants.gearRatio;

  /** Checking elapsed time for absolute calibration */
  private final Timer m_timer = new Timer();
  /** Last time when we resetted to absolute */
  private double lastAbsoluteTime;

  /**
   * Angle to go by the arm
   * unit: degrees
   */
  private double setpoint = 0;

  private final DutyCycleOut m_percentOut = new DutyCycleOut(0);

  /**
   * Output to set in open loop
   * unit: percent
   */
  private double targetOutput = 0;

  private ArmControlState armControlState = ArmControlState.HOLD;

  /** unit: rotations */
  private double lastDemandedRotation;


  //private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0, 0, 1, false, false, false);

  /** unit: rotations */
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);

  public enum ArmControlState {
    OPEN_LOOP,
    INTAKE,
    SPEAKER_SHORT,
    SPEAKER_LONG,
    AMP,
    HOLD,
    ZERO,
  }

  public ArmSubsystem() {
    motorConfig();
    lastDemandedRotation = getArmAngleFalcon();

    /** sets Bore reading to the desired "zero" position */
    boreEncoder.setPositionOffset(ArmConstants.positionOffset);

    m_timer.reset();
    m_timer.start();

    lastAbsoluteTime = m_timer.get();
    
  }

  public static ArmSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new ArmSubsystem();
    }
    return mInstance;
  }

  private void motorConfig() {
    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    TalonFXConfiguration configs = new TalonFXConfiguration();

    configs.Slot0.kP = ArmConstants.kP;
    configs.Slot0.kI = ArmConstants.kI;
    configs.Slot0.kD = ArmConstants.kD;
    configs.Slot0.kS = ArmConstants.kS;
    configs.Slot0.kV = ArmConstants.kV;

    configs.Voltage.PeakForwardVoltage = 12;
    configs.Voltage.PeakReverseVoltage = -12;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    
    configs.MotionMagic.MotionMagicAcceleration = ArmConstants.armAcceleration; 
    configs.MotionMagic.MotionMagicCruiseVelocity = ArmConstants.armCruiseVelocity; 
    //configs.CurrentLimits.StatorCurrentLimit = 300;
    //configs.CurrentLimits.StatorCurrentLimitEnable = true;
    //configs.CurrentLimits.SupplyCurrentLimit = 80;
    //configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    armMotor.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Arm Bore Degrees", Conversions.revolutionsToDegrees(getArmAngleBore()));
    // SmartDashboard.putNumber("Arm Falcon Degrees", Conversions.revolutionsToDegrees(getArmAngleFalcon()));
    // SmartDashboard.putBoolean("Arm At Setpoint", isAtSetpointFalcon());
    // SmartDashboard.putString("Arm State", armControlState.toString());
    // SmartDashboard.putBoolean("MotMag Working", armMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled);
    // SmartDashboard.putNumber("Last Demanded Rot", Conversions.revolutionsToDegrees(lastDemandedRotation));
    // SmartDashboard.putNumber("Motor Current", armMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber("Arm Voltage", armMotor.getMotorVoltage().getValueAsDouble());

    switch (armControlState) {
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case SPEAKER_SHORT:
        setArmAngleMotionMagic(ArmConstants.SPEAKER_SHORT);
        break;
      case SPEAKER_LONG:
        setArmAngleMotionMagic(ArmConstants.SPEAKER_LONG);
        break;
      case AMP:
        setArmAngleMotionMagic(ArmConstants.AMP);
        break;
      case INTAKE:
        setArmAngleMotionMagic(ArmConstants.INTAKE);
        break;
      case HOLD:
        armMotor.setControl(new NeutralOut());
        break;
      case ZERO:
        setArmAngleMotionMagic(ArmConstants.ZERO);
        break;
      default:
        setArmPercentOutput(0.0);
        break;
    }

    lastDemandedRotation = getArmAngleFalcon();

    autoCalibration();
  }

  /** resets Falcon encoder to zero */
  public void resetFalconEncoder() {
    armMotor.setPosition(0);
  }

  // resets falcon encoder to the bore reading so that bore and falcon have the same initial reading
  // theoretically, Falcon position / 119.5 = Bore encoder position + offset at all times
  public void resetToAbsolute() {
    double angle = getArmAngleBore();
    var position = armGearbox.drivenToDriving(angle);
    armMotor.setPosition(position);
    lastAbsoluteTime = m_timer.get();
  }


  /** @return true if within angle tolerance */
  public boolean isAtSetpointFalcon() {
    return Math.abs(getArmAngleFalcon() - Conversions.degreesToRevolutions(setpoint)) < ArmConstants.angleTolerance;
  }

  public boolean isAtZeroFalcon() {
    return armGearbox.drivingToDriven(armMotor.getRotorPosition().getValue()) < ArmConstants.angleTolerance;
  }

  /** @return setpoint unit: degrees */
  public double getSetpoint() {
    return setpoint;
  }

  /** basically "zeros" arm */
  public double zeroSetpoint() {
    return setpoint = 0;
  }

  /** unit: revolutions */
  public double getArmAngleFalcon() {
    return armGearbox.drivingToDriven(armMotor.getPosition().getValueAsDouble());
  }

  /** unit: revolutions */
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition() - boreEncoder.getPositionOffset();
  }

  // TODO: add max/min angles here!
  public void setArmAngleMotionMagic(double target) {
    setpoint = target;
    armMotor.setControl(motionMagicVoltage.withPosition(
      armGearbox.drivenToDriving(Conversions.degreesToRevolutions(setpoint))
    ));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
    lastDemandedRotation = getArmAngleFalcon();
  }

  public void setArmPercentOutput(double percent) {
    armControlState = ArmControlState.OPEN_LOOP;
    targetOutput = percent;
    armMotor.setControl(m_percentOut.withOutput(targetOutput));
    lastDemandedRotation = getArmAngleFalcon();
  }

  public void hold() {
    armControlState = ArmControlState.HOLD;
    armMotor.setControl(new NeutralOut());
  }

  /** unit: rotation */
  public void updateLastDemandedRotation(double rotation) {
    lastDemandedRotation = rotation;
  }

  public void setArmControlState(ArmControlState state) {
    armControlState = state;
  }

  public ArmControlState getArmControlState() {
    return armControlState;
  }

  public void setMotorOutput() {
      armMotor.setControl(m_percentOut.withOutput(targetOutput));
  }

  public TalonFX getArmMotor() {
    return armMotor;
  }

  public double getLastDemandedRotation() {
    return lastDemandedRotation;
  }
 
  /** Resets to absolute if:
   * time has elapsed 10 seconds since previous calibration
   * OR
   * current Falcon rotation is 0.5 degrees off from the Bore reading
   * AND
   * the mechanism isn't moving
   */
  public void autoCalibration(){
    boolean timerCondition = m_timer.get() - lastAbsoluteTime > 10;
    boolean angleCondition = Math.abs(getArmAngleBore() - getArmAngleFalcon()) >= Conversions.degreesToRevolutions(0.5);
    boolean speedCondition = Math.abs(armMotor.getRotorVelocity().getValueAsDouble()) < 0.005;
    if((timerCondition || angleCondition) && speedCondition) {
      resetToAbsolute();
      lastAbsoluteTime = m_timer.get();
    }
  }
}
