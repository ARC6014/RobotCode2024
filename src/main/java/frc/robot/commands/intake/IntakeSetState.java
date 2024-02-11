package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Running;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

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
        mIntakeSubsystem.setState(Running.NEUTRAL);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
