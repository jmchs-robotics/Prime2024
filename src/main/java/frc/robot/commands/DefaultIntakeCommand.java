package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {
    private final IntakeSubsystem m_intakesubsystem;

    public DefaultIntakeCommand(IntakeSubsystem intakesubsystem) {
        m_intakesubsystem = intakesubsystem;
        addRequirements(intakesubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_intakesubsystem.stopIntake();
        m_intakesubsystem.stopIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
