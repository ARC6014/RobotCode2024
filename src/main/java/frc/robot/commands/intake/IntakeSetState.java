package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSetState extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private IntakeSubsystem.Running mState;

    public IntakeSetState(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Running state) {
        mIntakeSubsystem = intakeSubsystem;
        mState = state;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void initialize() {
        mIntakeSubsystem.setState(mState);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return mIntakeSubsystem.isAtSetpoint();
    }
}
