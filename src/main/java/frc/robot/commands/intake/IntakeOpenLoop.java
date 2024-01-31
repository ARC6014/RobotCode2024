package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Running;

public class IntakeOpenLoop extends Command {
    private IntakeSubsystem mIntakeSubsystem;
    private DoubleSupplier mOutputSupplier;

    public IntakeOpenLoop(IntakeSubsystem intakeSubsystem, DoubleSupplier outputSupplier) {
        mIntakeSubsystem = intakeSubsystem;
        mOutputSupplier = outputSupplier;
        addRequirements(mIntakeSubsystem);
    }

    @Override
    public void initialize() {
        mIntakeSubsystem.setState(Running.OPENLOOP);
    }

    @Override
    public void execute() {
        double output = mOutputSupplier.getAsDouble();
        if (output > 0.04 || output < -0.04) {
            mIntakeSubsystem.setOpenLoop(output);
        }
    }

    @Override
    public void end(boolean interrupted) {
        mIntakeSubsystem.setOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
