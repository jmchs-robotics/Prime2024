package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootForwardTurbo extends Command {

    private Shooter m_shooter;
    private Intake m_intake;
    Timer timer = new Timer();

    public ShootForwardTurbo(Shooter shooter, Intake intake) {

        m_shooter = shooter;
        m_intake = intake;
        addRequirements(m_shooter, m_intake);
    
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.get() < 1) {
            m_shooter.setBothShooterMotors(0.9);
        } else {
            m_shooter.setBothShooterMotors(0.9);
            m_intake.setIndex(0.4);
            m_intake.setIntake(0.15);
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stopBothShooterMotors();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
