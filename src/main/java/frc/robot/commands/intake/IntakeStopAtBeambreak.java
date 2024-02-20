package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopAtBeambreak extends Command {
    private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    @Override
    public void initialize() {}

    @Override
    public void execute() {}
  
    @Override
    public void end(boolean interrupted) {}
  
    @Override
    public boolean isFinished() {
        // beambreak infrared light is broken, so it is false when object is detected
        return !mIntakeSubsystem.getBeambreak();
    }
}
