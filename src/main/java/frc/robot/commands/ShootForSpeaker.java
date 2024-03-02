package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootForSpeaker extends Command {

    private Shooter m_subsystem;

    public ShootForSpeaker(Shooter subsystem) {

        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_subsystem.setBothShooterMotors(0.7);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopBothShooterMotors();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
