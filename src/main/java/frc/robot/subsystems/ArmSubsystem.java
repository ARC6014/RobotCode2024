// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.team6014.lib.math.Conversions;
import frc.team6014.lib.math.Gearbox;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final TalonFX armMotor = new TalonFX(ArmConstants.motorID, Constants.CANIVORE_CANBUS);
  private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(ArmConstants.boreChannel);

  private static ArmSubsystem mInstance;
  public Gearbox armGearbox = ArmConstants.gearRatio;

  /** unit: degree */
  private double setpoint = 0; // angle to go by arm
  // TODO: FOC is false for now
  private final DutyCycleOut m_percentOut = new DutyCycleOut(0, false, false, false, false);

  /** Output to set in open loop (percent) */
  private double targetOutput = 0;
  private ArmControlState armControlState = ArmControlState.OPEN_LOOP;

  /** unit: degree */
  private double lastDemandedRotation;

  private final MotionMagicTorqueCurrentFOC motionMagic = new MotionMagicTorqueCurrentFOC(0, 0, 1, false, false, false);

  /** unit: rot */
  private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0, false, 0, 0, false, false, false);

  public enum ArmControlState {
    OPEN_LOOP,
    MOTION_MAGIC,
  }

  public ArmSubsystem() {
    motorConfig();
    lastDemandedRotation = getArmAngleFalcon();
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
    // Slot0 is for MotionMagicVoltage
    configs.Slot0.kP = ArmConstants.kP;
    configs.Slot0.kI = ArmConstants.kI;
    configs.Slot0.kD = ArmConstants.kD;
    configs.Slot0.kS = ArmConstants.kS;
    configs.Slot0.kV = ArmConstants.kV;

    configs.Voltage.PeakForwardVoltage = 10;
    configs.Voltage.PeakReverseVoltage = -10;
    configs.TorqueCurrent.PeakForwardTorqueCurrent = 180;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = 180;
    ;
    configs.MotionMagic.MotionMagicAcceleration = 70; // change
    configs.MotionMagic.MotionMagicCruiseVelocity = 50; // change

    armMotor.setInverted(ArmConstants.motorInverted);
    armMotor.setNeutralMode(ArmConstants.neutralMode);

    configs.CurrentLimits.StatorCurrentLimit = 300;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimit = 80;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    armMotor.getConfigurator().apply(configs);

    resetFalconEncoder();
    resetToAbsolute();
    // might not need this as mech is not linear
    // boreEncoder.setDistancePerRotation(ArmConstants.distancePerRotation);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Arm Bore Angle", getArmAngleBore());
    // SmartDashboard.putNumber("Arm Falcon Angle", getArmAngleFalcon());
    // SmartDashboard.putString("Arm State: ", armControlState.toString());
    // SmartDashboard.putNumber("Last Demanded Angle", lastDemandedRotation);

    switch (armControlState) {
      case OPEN_LOOP:
        setMotorOutput();
        break;
      case MOTION_MAGIC:
        setArmAngleMotionMagic();
        break;
      default:
        setArmPercentOutput(0.0);
        break;
    }

    lastDemandedRotation = getArmAngleFalcon();

    SmartDashboard.putBoolean("MotMag Working", armMotor.getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled);
    SmartDashboard.putNumber("Falcon Voltage", armMotor.getMotorVoltage().getValueAsDouble());

  }

  // resets falcon encoder to some pre-set zero position
  public void resetFalconEncoder() {
    armMotor.setPosition(ArmConstants.resetAngle * armGearbox.getRatio());
  }

  /* Conversions */
  public double drivenToDriver(double revolutions) {
    return revolutions * (1.0 / armGearbox.getRatio());
  }

  public double driverToDriven(double revolutions) {
    return armGearbox.calculate(revolutions);
  }

  // resets falcon encoder so that bore and falcon have the same initial reading
  // theoretically, Falcon position / 119.5 = Bore encoder position + offset at
  // all times
  public void resetToAbsolute() {
    double position = drivenToDriver(getArmAngleBore() + ArmConstants.positionOffset);
    armMotor.setPosition(position);
  }

  /** @return true if within angle tolerance */
  public boolean isAtSetpointFalcon() {
    return Math.abs(getArmAngleFalcon() - setpoint) < ArmConstants.angleTolerance;
  }

  public boolean isAtZeroFalcon() {
    return armMotor.getRotorPosition().getValue() / armGearbox.getRatio() == ArmConstants.resetAngle;
  }

  /** @return setpoint (degree) */
  public double getSetpoint() {
    return setpoint;
  }

  // TODO: Check return unit type
  public double getArmAngleFalcon() {
    return armMotor.getRotorPosition().getValue() / armGearbox.getRatio();

  }

  // TODO: Do we need gear ratio here?
  public double getArmAngleBore() {
    return boreEncoder.getAbsolutePosition() + ArmConstants.positionOffset;
  }

  // TODO: add max/min angles here!
  // TODO: Check rad-angle conversion
  // When using an absolute sensor, such as a CANcoder, the sensor offset must be
  // configured such that a position of 0 represents the arm being held
  // horizontally forward. From there, the RotorToSensor ratio must be configured
  // to the ratio between the absolute sensor and the Talon FX rotor.
  public void setArmAngleMotionMagic() {
    armMotor.setControl(motionMagicVoltage.withPosition(setpoint * armGearbox.getRatio())
        .withFeedForward(ArmConstants.kG * Math.cos(Conversions.revolutionsToRadians(getArmAngleBore()))));
  }

  public void setArmVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmPercentOutput(double percent) {
    if (armControlState != ArmControlState.OPEN_LOOP) {
      armControlState = ArmControlState.OPEN_LOOP;
    }
    targetOutput = percent;
    lastDemandedRotation = getArmAngleFalcon();
  }

  // sets arm to the respective setpoint
  public void setArmPosition(double target) {
    if (armControlState != ArmControlState.MOTION_MAGIC) {
      armControlState = ArmControlState.MOTION_MAGIC;
    }
    setpoint = target;
    lastDemandedRotation = getArmAngleFalcon();
  }

  /** unit: degree */
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

}
