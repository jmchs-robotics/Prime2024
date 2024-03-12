package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

public class LimelightAiming extends Command{

    private double kP = 0.01;
    private DriveSubsystem m_drive;

    public LimelightAiming(DriveSubsystem drive) {

        m_drive = drive;
        addRequirements(m_drive);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double targetAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        targetAngularVelocity *= -1;

        m_drive.drive(0, 0, targetAngularVelocity, false, true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
