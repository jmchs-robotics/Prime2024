package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootForAmp extends Command {
    private ShooterSubsystem m_shootersubsystem;

    public ShootForAmp(ShooterSubsystem shootersubsystem) {
        m_shootersubsystem = shootersubsystem;
        addRequirements(shootersubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shootersubsystem.setBothShooterMotors(0.17);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shootersubsystem.stopBothShooterMotors();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }   
}
