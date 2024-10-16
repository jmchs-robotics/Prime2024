package frc.robot.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.intake.IntakeSubsystem;

public class ShooterCommands {

    private ShooterCommands() {}

    public static Command shootSpeaker(ShooterSubsystem shooter, IntakeSubsystem intake) {

        return Commands.runEnd(
            () -> {
                Timer timer = new Timer();
                timer.start();
                timer.reset();
                while (timer.get() < 1) {
                    shooter.setBothShooterMotors(0.95);
                    intake.setIndex(0.7);
                }
                shooter.setBothShooterMotors(0.95);
                intake.setIndex(0.7);
                intake.setIntake(0.4);
            }, 
            () -> {
                shooter.stopBothShooterMotors();
                intake.stopIndex();
                intake.stopIntake();
            }, 
            shooter, intake);

    }

    public static Command reverseShooter(ShooterSubsystem shooter) {

        return Commands.runEnd(
            () -> {shooter.setBothShooterMotors(-0.2);}, 
            () -> {shooter.stopBothShooterMotors();}, 
            shooter);
            
    }
    
}
