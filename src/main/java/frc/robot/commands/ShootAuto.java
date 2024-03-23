package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Robot;

public class ShootAuto extends Command {
    private ShooterSubsystem m_shootersubsystem;
    private IntakeSubsystem m_intakesubsystem;
    Timer timer = new Timer();

    public ShootAuto(ShooterSubsystem shootersubsystem, IntakeSubsystem intakesubsystem) {
        m_shootersubsystem = shootersubsystem;
        m_intakesubsystem = intakesubsystem;
        addRequirements(shootersubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (true) {
            if (timer.get() < 1) {
                m_shootersubsystem.setBothShooterMotors(0.9);
            } else {
                m_shootersubsystem.setBothShooterMotors(0.9);
                m_intakesubsystem.setIndex(0.7);
                m_intakesubsystem.setIntake(0.4);
            }
        } else {
            m_shootersubsystem.setBothShooterMotors(0);
        }
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