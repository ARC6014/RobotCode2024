package frc.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.ArmSubsystem;
import frc.shuffleboard.ShuffleboardTabBase;

public class OperatorTab extends ShuffleboardTabBase{

    /* ARM SUBSYSTEM */
    private ArmSubsystem mArm = ArmSubsystem.getInstance();
    private GenericEntry boreAngle, falconAngle, armState, atSetpoint, atZero;
    


    public OperatorTab() {
        super();
    }

    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Operator");

        boreAngle = mTab
        .add("Bore Angle", 0)
        .withPosition(0, 0)
        .withSize(1, 1)
        .getEntry();

        falconAngle = mTab
        .add("Falcon Angle", 0)
        .withPosition(0, 1)
        .withSize(1, 1)
        .getEntry();

        armState = mTab
        .add("ArmState", "NaS")
        .withPosition(0, 2)
        .withSize(1, 1)
        .getEntry();

        atSetpoint = mTab
        .add("At Setpoint", false)
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

    }

    @Override
    public void update() {
        boreAngle.setDouble(truncate(mArm.getArmAngleBore()));
        falconAngle.setDouble(truncate(mArm.getArmAngleFalcon()));
        armState.setString(mArm.getArmControlState().toString());
        atSetpoint.setBoolean(mArm.isAtSetpointFalcon());
        atZero.setBoolean(mArm.isAtZeroFalcon());



    }



    
}
