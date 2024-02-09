package frc.shuffleboard.tabs;

import com.ctre.phoenix6.signals.MotionMagicIsRunningValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CANdleLed;
import frc.shuffleboard.ShuffleboardTabBase;
import frc.team6014.lib.math.Conversions;

public class OperatorTab extends ShuffleboardTabBase {

    /* ARM SUBSYSTEM */
    private ArmSubsystem mArm = ArmSubsystem.getInstance();
    private GenericEntry boreAngle, falconAngle, armState, atSetpoint, atZero, motMagStatus, lastRotation, current,
            voltage;

    /* LEDS */
    // private CANdleLed mLed = CANdleLed.getInstance();
    private GenericEntry currentAnimation;

    public OperatorTab() {
        super();
    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");

        boreAngle = mTab
                .add("A-Bore Angle", 0)
                .withPosition(0, 0)
                .withSize(1, 1)
                .getEntry();

        falconAngle = mTab
                .add("A-Falcon Angle", 0)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();

        armState = mTab
                .add("ArmState", "NaS")
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();

        atSetpoint = mTab
                .add("A-At Setpoint", false)
                .withPosition(1, 0)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withSize(1, 1)
                .getEntry();

        atZero = mTab
                .add("At Zero", false)
                .withPosition(1, 1)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();

        motMagStatus = mTab
                .add("A-MMagic Working", false)
                .withPosition(1, 2)
                .withSize(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox)
                .getEntry();

        lastRotation = mTab
                .add("A-Last Dem. Rotation", 0)
                .withPosition(2, 0)
                .withSize(1, 1)
                .getEntry();

        current = mTab
                .add("A-Current", 0)
                .withPosition(2, 1)
                .withSize(1, 1)
                .getEntry();

        voltage = mTab
                .add("A-Voltage", 0)
                .withPosition(2, 2)
                .withSize(1, 1)
                .getEntry();

        // currentAnimation = mTab
        // .add("Current Anim", "NONE")
        // .withPosition(2, 0)
        // .withSize(1, 1)
        // .getEntry();
    }

    @Override
    public void update() {
        /* ARM */
        boreAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mArm.getArmAngleBore())));
        falconAngle.setDouble(truncate(Conversions.revolutionsToDegrees(mArm.getArmAngleFalcon())));
        armState.setString(mArm.getArmControlState().toString());
        atSetpoint.setBoolean(mArm.isAtSetpointFalcon());
        atZero.setBoolean(mArm.isAtZeroFalcon());
        motMagStatus.setBoolean(
                mArm.getArmMotor().getMotionMagicIsRunning().getValue() == MotionMagicIsRunningValue.Enabled);
        lastRotation.setDouble(Conversions.revolutionsToDegrees(mArm.getLastDemandedRotation()));
        current.setDouble(mArm.getArmMotor().getStatorCurrent().getValueAsDouble());
        voltage.setDouble(mArm.getArmMotor().getMotorVoltage().getValueAsDouble());

        /* LEDS */
        // currentAnimation.setString(mLed.getCurrentAnimation().toString());
    }

}
