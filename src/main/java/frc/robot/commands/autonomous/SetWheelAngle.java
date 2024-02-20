package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.DriveSubsystem;

/** 
 * Point all the wheels toward a given angle.  Don't drive anywhere or move the chassis at all.
 */
public class SetWheelAngle extends Command {

    private final DriveSubsystem drivetrain;
    private final double angleWant;

    public SetWheelAngle(DriveSubsystem drivetrain, double a) {
        this.drivetrain = drivetrain;
        this.angleWant = a;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {   
        SwerveModuleState[] wheelStates = {
            new SwerveModuleState(0, Rotation2d.fromDegrees(angleWant)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angleWant)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angleWant)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(angleWant))
        };

        drivetrain.setModuleStates(wheelStates);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end( boolean interrupted) {}

}