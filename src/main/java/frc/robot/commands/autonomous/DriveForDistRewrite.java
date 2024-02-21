package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class DriveForDistRewrite extends Command {

    private final DriveSubsystem m_drive;

    private final double distForward;
    private final double distStrafe;
    private final double distance;
    private final double angle;

    public DriveForDistRewrite(double dForward, DriveSubsystem drivetrain) {
        this(dForward, 0, drivetrain);
    }

    public DriveForDistRewrite(double dForward, double dStrafe, DriveSubsystem drivetrain) {
        distForward = dForward;
        distStrafe = dStrafe;
        m_drive = drivetrain;

        distance = Math.sqrt((dForward * dForward) + (dStrafe + dStrafe));
        angle = Math.toDegrees(Math.atan2(dStrafe, dForward));

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        // Set their angle at 0 speed first so it can align better
        m_drive.setModuleStates(0, angle);
        m_drive.resetEncoders();
    }

    @Override
    public void execute() {
        m_drive.setModuleStates(1, angle);
    }

    @Override
    public boolean isFinished() {
        double endTicks = m_drive.getSwerveModule(0).inchesToEncoderTicks(distance);
        double currentTicks = m_drive.getSwerveModule(0).getPositionTicks();

        if (endTicks - currentTicks <= 0) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setModuleStates(0, 0);
    }
    
}
