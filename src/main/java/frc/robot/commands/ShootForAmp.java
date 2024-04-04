package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootForAmp extends Command {
    private ShooterSubsystem m_shootersubsystem;
    private IntakeSubsystem m_intakesubsystem;
    private Timer timer = new Timer();

    public ShootForAmp(ShooterSubsystem shootersubsystem, IntakeSubsystem intakesubsystem) {
        m_shootersubsystem = shootersubsystem;
        m_intakesubsystem = intakesubsystem;
        addRequirements(shootersubsystem, intakesubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.get() < 0.2) {
            m_shootersubsystem.setBothShooterMotors(0.17);
        } else {
            m_shootersubsystem.setBothShooterMotors(0.17);
            m_intakesubsystem.setIntake(0.15);
            m_intakesubsystem.setIndex(0.2);
        }
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shootersubsystem.stopBothShooterMotors();
        m_intakesubsystem.stopIndex();
        m_intakesubsystem.stopIntake();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }   
}
