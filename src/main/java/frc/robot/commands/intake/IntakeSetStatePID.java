package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Running;
import frc.robot.subsystems.ShooterSubsystem.ShooterState;

public class IntakeSetStatePID extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private IntakeSubsystem.Running mState;

    public IntakeSetStatePID(IntakeSubsystem intakeSubsystem, IntakeSubsystem.Running state) {
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
        // mIntakeSubsystem.setState(Running.S_DOWN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
