package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Running;

public class IntakeSetOpenLoop extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private double percent;

    public IntakeSetOpenLoop(IntakeSubsystem intakeSubsystem, double output) {
        mIntakeSubsystem = intakeSubsystem;
        percent = output;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void initialize() {
        mIntakeSubsystem.setState(Running.OPENLOOP);
    }

    @Override
    public void execute() {
        mIntakeSubsystem.setOpenLoop(percent);
    }

    @Override
    public void end(boolean interrupted) {
        // mIntakeSubsystem.setState(Running.S_DOWN);
        mIntakeSubsystem.setOpenLoop(0.0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
