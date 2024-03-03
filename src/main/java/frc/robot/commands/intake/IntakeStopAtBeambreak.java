package frc.robot.commands.intake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeStopAtBeambreak extends Command {
    private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    Debouncer m_debouncer = new Debouncer(0.3, Debouncer.DebounceType.kFalling);
    private boolean hasEnded = false;

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        hasEnded = m_debouncer.calculate(mIntakeSubsystem.getBeambreak());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {

        // beambreak infrared light is broken, so it is false when object is detected
        return !hasEnded;
    }
}
