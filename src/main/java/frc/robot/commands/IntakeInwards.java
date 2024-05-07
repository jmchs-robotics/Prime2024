package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeInwards extends Command {
    private final IntakeSubsystem m_intakesubsystem;

    public IntakeInwards (IntakeSubsystem intakesubsystem) {
        m_intakesubsystem = intakesubsystem;
        addRequirements(intakesubsystem);
    }

    @Override
    public void initialize () {}

    @Override
    public void execute () {
        m_intakesubsystem.setIntake(0.15);
        m_intakesubsystem.setIndex(0.2);
    }

    @Override
    public boolean isFinished () {
        if (m_intakesubsystem.isUsingBeamBreak()) {
            return m_intakesubsystem.isBeamBreakTripped();
        } else {
            return false;
        }
    }

    @Override
    public void end (boolean interrupted) {
        if (m_intakesubsystem.isUsingBeamBreak() && m_intakesubsystem.isBeamBreakTripped()) {
            Timer timer = new Timer();
            timer.start();
            timer.reset();
            while (timer.get() < 0.1) {
                m_intakesubsystem.setIntake(-0.15);
                m_intakesubsystem.setIndex(-0.2);
            }
        }

        m_intakesubsystem.stopIntake();
        m_intakesubsystem.stopIndex();
    }
}
