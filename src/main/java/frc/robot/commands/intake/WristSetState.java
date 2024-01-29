package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.WristSubsystem;

public class WristSetState extends Command {
    private WristSubsystem mWristSubsystem;
    private WristSubsystem.Position mState;

    public WristSetState(WristSubsystem wristSubsystem, WristSubsystem.Position state) {
        mWristSubsystem = wristSubsystem;
        mState = state;
        addRequirements(mWristSubsystem);
    }

    @Override
    public void initialize() {
        mWristSubsystem.setState(mState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return mWristSubsystem.isAtSetpoint();
    }
}
