package frc.robot.commands;

import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOutwards extends Command {
    
    private final Intake m_subsystem;

    public IntakeOutwards(Intake subsystem) {

        m_subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize () {}

    @Override
    public void execute () {
        m_subsystem.setIntake(-0.2);
    }

    @Override
    public boolean isFinished () {
        return false;
    }

    @Override
    public void end (boolean interrupted) {
        m_subsystem.stopIntake();
    }
    
}
