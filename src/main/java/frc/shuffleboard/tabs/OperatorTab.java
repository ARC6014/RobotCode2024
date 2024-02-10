package frc.shuffleboard.tabs;

import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CANdleLed;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.shuffleboard.ShuffleboardTabBase;
import frc.team6014.lib.math.Conversions;

public class OperatorTab extends ShuffleboardTabBase {

        /* ARM SUBSYSTEM */
        private ArmSubsystem mArm = ArmSubsystem.getInstance();
        private GenericEntry armBoreAngle, armFalconAngle, armState, armAtSetpoint, armAtZero, armMotMagStatus,
                        lastRotation, current,
                        voltage;

        /* WRIST+INTAKE SUBSYSTEMS */
        private WristSubsystem mWrist = WristSubsystem.getInstance();
        private IntakeSubsystem mIntake = IntakeSubsystem.getInstance();
        private GenericEntry wristBoreAngle, wristFalconAngle, wristState, wristAtSetpoint, intakeState, intakeSetpoint,
                        intakeActualVelocity, intakeVoltage, wristMMRunning;

        /* SHOOTER SUBSYSTEM */
        private ShooterSubsystem mShooter = ShooterSubsystem.getInstance();
        private GenericEntry shooterBeamBreak, shooterTarget, shooterActualRPM, shooterState, pdhVoltage, smartVoltage;
        /* LEDS */
        // private CANdleLed mLed = CANdleLed.getInstance();
        private GenericEntry currentAnimation;

        public OperatorTab() {
                super();
        }

        @Override
        public void createEntries() {
                mTab = Shuffleboard.getTab("Operator");

                armBoreAngle = mTab
                                .add("A-Bore Angle", 0)
                                .withPosition(0, 0)
                                .withSize(1, 1)
                                .getEntry();

                armFalconAngle = mTab
                                .add("A-Falcon Angle", 0)
                                .withPosition(1, 0)
                                .withSize(1, 1)
                                .getEntry();

                armState = mTab
                                .add("ArmState", "NaS")
                                .withPosition(2, 0)
                                .withSize(1, 1)
                                .getEntry();

                armAtSetpoint = mTab
                                .add("A-At Setpoint", false)
                                .withPosition(3, 0)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .withSize(1, 1)
                                .getEntry();

                armAtZero = mTab
                                .add("At Zero", false)
                                .withPosition(4, 0)
                                .withSize(1, 1)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();

                armMotMagStatus = mTab
                                .add("A-MMagic Working", false)
                                .withPosition(0, 1)
                                .withSize(1, 1)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();

                lastRotation = mTab
                                .add("A-Last Dem. Rotation", 0)
                                .withPosition(1, 1)
                                .withSize(1, 1)
                                .getEntry();

                // current = mTab
                // .add("A-Current", 0)
                // .withPosition(7, 0)
                // .withSize(1, 1)
                // .getEntry();

                // voltage = mTab
                // .add("A-Voltage", 0)
                // .withPosition(8, 0)
                // .withSize(1, 1)
                // .getEntry();

                // currentAnimation = mTab
                // .add("Current Anim", "NONE")
                // .withPosition(2, 0)
                // .withSize(1, 1)
                // .getEntry();

                wristBoreAngle = mTab
                                .add("W-Bore Angle", 0)
                                .withPosition(2, 1)
                                .withSize(1, 1)
                                .getEntry();

                wristFalconAngle = mTab
                                .add("W-Falcon Angle", 0)
                                .withPosition(3, 1)
                                .withSize(1, 1)
                                .getEntry();

                wristState = mTab
                                .add("WristState", "NaS")
                                .withPosition(4, 1)
                                .withSize(1, 1)
                                .getEntry();

                wristAtSetpoint = mTab
                                .add("W-At Setpoint", false)
                                .withPosition(0, 2)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .withSize(1, 1)
                                .getEntry();

                intakeState = mTab
                                .add("IntakeState", "NaS")
                                .withPosition(1, 2)
                                .withSize(1, 1)
                                .getEntry();

                intakeSetpoint = mTab
                                .add("I-Setpoint", 0)
                                .withPosition(2, 2)
                                .withSize(1, 1)
                                .getEntry();

                shooterBeamBreak = mTab
                                .add("S-BeamBreak", true)
                                .withPosition(3, 2)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .withSize(1, 1)
                                .getEntry();

                shooterTarget = mTab
                                .add("S-RPM", 0)
                                .withPosition(4, 2)
                                .withSize(1, 1)
                                .getEntry();

                shooterActualRPM = mTab
                                .add("S-RPM Actual", 0)
                                .withPosition(0, 3)
                                .withSize(1, 1)
                                .getEntry();

                shooterState = mTab
                                .add("ShooterState", "NaS")
                                .withPosition(1, 3)
                                .withSize(1, 1)
                                .getEntry();

                pdhVoltage = mTab
                                .add("PDH (V)", 0)
                                .withPosition(2, 3)
                                .withSize(1, 1)
                                .getEntry();

                smartVoltage = mTab
                                .add("S-Smart Voltage", 0)
                                .withPosition(3, 3)
                                .withSize(1, 1)
                                .getEntry();
                intakeVoltage = mTab
                                .add("I-Voltage", 0)
                                .withPosition(4, 3)
                                .withSize(1, 1)
                                .getEntry();

                intakeActualVelocity = mTab
                                .add("I-Actual Vel", 0)
                                .withPosition(0, 4)
                                .withSize(1, 1)
                                .getEntry();
                wristMMRunning = mTab
                                .add("W-MMagic Working", false)
                                .withPosition(1, 4)
                                .withSize(1, 1)
                                .withWidget(BuiltInWidgets.kBooleanBox)
                                .getEntry();

        }

        @Override
        public void update() {
                /* ARM */
                armBoreAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mArm.getArmAngleBore())));
                armFalconAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mArm.getArmAngleFalcon())));
                armState.setString(mArm.getArmControlState().toString());
                armAtSetpoint.setBoolean(mArm.isAtSetpointFalcon());
                armAtZero.setBoolean(mArm.isAtZeroFalcon());
                armMotMagStatus.setBoolean(
                                mArm.getArmMotor().getMotionMagicIsRunning()
                                                .getValue() == MotionMagicIsRunningValue.Enabled);
                lastRotation.setDouble(Conversions.revolutionsToDegrees(mArm.getLastDemandedRotation()));
                // current.setDouble(mArm.getArmMotor().getStatorCurrent().getValueAsDouble());
                // voltage.setDouble(mArm.getArmMotor().getMotorVoltage().getValueAsDouble());

                /* WRIST+INTAKE */
                wristBoreAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mWrist.getBoreEncoderPosition())));
                wristFalconAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mWrist.getFalconPosition())));
                wristState.setString(mWrist.getState().toString());
                wristAtSetpoint.setBoolean(mWrist.isAtSetpoint());
                wristMMRunning.setBoolean(
                                mWrist.getMotor().getMotionMagicIsRunning()
                                                .getValue() == MotionMagicIsRunningValue.Enabled);

                intakeState.setString(mIntake.getState().toString());
                intakeSetpoint.setDouble(truncate(mIntake.getSetpoint()));
                intakeActualVelocity.setDouble(truncate(mIntake.getVelocity()));
                intakeVoltage.setDouble(truncate(mIntake.getVoltage()));

                /* SHOOTER */
                shooterBeamBreak.setBoolean(mShooter.getSensorState());
                shooterTarget.setDouble(truncate(mShooter.getTargetRPM()));
                shooterActualRPM.setDouble(truncate(mShooter.getMasterMotorSpeed()));
                shooterState.setString(mShooter.getShooterState().toString());
                pdhVoltage.setDouble(truncate(mShooter.getPDHVoltage()));
                smartVoltage.setDouble(truncate(mShooter.getSmartVoltageShooter()));

                /* LEDS */
                // currentAnimation.setString(mLed.getCurrentAnimation().toString());
        }

}
