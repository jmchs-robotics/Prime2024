package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class DefaultShooterCommand extends Command {

    private final Shooter m_subsystem;

    public DefaultShooterCommand(Shooter subsystem) {

        m_subsystem = subsystem;
        addRequirements(m_subsystem);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
