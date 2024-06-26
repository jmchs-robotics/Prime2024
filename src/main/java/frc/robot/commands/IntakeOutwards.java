package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOutwards extends Command {
    private final IntakeSubsystem m_intakesubsystem;

    public IntakeOutwards(IntakeSubsystem intakesubsystem) {
        m_intakesubsystem = intakesubsystem;
        addRequirements(intakesubsystem);
    }

    @Override
    public void initialize () {}

    @Override
    public void execute () {
        m_intakesubsystem.setIntake(-0.15);
        m_intakesubsystem.setIndex(-0.2);
    }

    @Override
    public boolean isFinished () {
        return false;
    }

    @Override
    public void end (boolean interrupted) {
        m_intakesubsystem.stopIntake();
        m_intakesubsystem.stopIndex();
    }
}
