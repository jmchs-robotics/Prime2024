package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class DefaultShooterCommand extends Command {
    private final ShooterSubsystem m_shootersubsystem;

    public DefaultShooterCommand(ShooterSubsystem shootersubsystem) {
        m_shootersubsystem = shootersubsystem;
        addRequirements(shootersubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shootersubsystem.stopBothShooterMotors();
    }

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
