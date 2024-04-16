package frc.robot.commands;

import frc.robot.subsystems.AmpSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class FlipAmpForward extends Command {

    private AmpSubsystem m_ampSubsystem;
    
    public FlipAmpForward(AmpSubsystem ampsubsystem) {
        m_ampSubsystem = ampsubsystem;
        addRequirements(ampsubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_ampSubsystem.setAmpMotor(0.1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_ampSubsystem.stopAmpMotor();
    }
}
