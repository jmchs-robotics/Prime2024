package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class DefaultIntakeCommand extends Command {

    private final Intake m_subsystem;

    public DefaultIntakeCommand(Intake subsystem) {

        m_subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsystem.stopIntake();
        m_subsystem.stopIndex();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
