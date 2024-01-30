package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IndexInwards extends Command {
    
    private final Intake m_subsystem;

    public IndexInwards(Intake subsystem) {

        m_subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize () {}

    @Override
    public void execute () {
        m_subsystem.indexPiece(0.5);
    }

    @Override
    public boolean isFinished () {
        return false;
    }

    @Override
    public void end (boolean interrupted) {}
    
}
