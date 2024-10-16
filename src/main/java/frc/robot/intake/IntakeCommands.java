package frc.robot.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    private IntakeCommands() {}

    public static Command intakeInwards(IntakeSubsystem intake) {

        return Commands.runEnd(
            () -> {
                intake.setIntake(0.15);
                intake.setIndex(0.2);
            }, 
            () -> {
                if (intake.isUsingBeamBreak()) {
                    Timer timer = new Timer();
                    timer.start();
                    timer.reset();
                    while(timer.get() < 0.1) {
                        intake.setIntake(-0.15);
                        intake.setIndex(-0.2);
                    }
                    intake.stopIntake();
                    intake.stopIndex();
                }
            }, intake)
                .until(() -> intake.isUsingBeamBreak()
                    ? intake.isBeamBreakTripped()
                    : false);

    }

    public static Command intakeOutwards(IntakeSubsystem intake) {

        return Commands.runEnd(
            () -> {
                intake.setIntake(-0.15);
                intake.setIndex(-0.2);
            }, 
            () -> {
                intake.stopIntake();
                intake.stopIndex();
            }, 
            intake);

    }
    
}
