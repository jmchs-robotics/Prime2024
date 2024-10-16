package frc.robot.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class DriveCommands {

    private static final double DEADBAND = 0.15;

    private DriveCommands() {}

    public static Command driveFieldRelative(
        DriveSubsystem driveSubsystem,
        double xSpeed,
        double ySpeed,
        double rotSpeed) {

            return Commands.run(
                () -> {driveSubsystem.drive(
                    -MathUtil.applyDeadband(xSpeed, DEADBAND), 
                    -MathUtil.applyDeadband(ySpeed, DEADBAND), 
                    -MathUtil.applyDeadband(rotSpeed, DEADBAND), 
                    true, 
                    true);},
                driveSubsystem);
        
    }
    
}
