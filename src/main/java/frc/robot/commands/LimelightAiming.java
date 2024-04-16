package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.OIConstants;

public class LimelightAiming extends Command {

    private double kP = 0.015;
    private DriveSubsystem m_drive;
    private XboxController m_controller;

    public LimelightAiming(DriveSubsystem drive, XboxController controller) {

        m_drive = drive;
        m_controller = controller;
        addRequirements(m_drive);

    }

    @Override
    public void initialize() {
        LimelightHelpers.setLEDMode_ForceOn("limelight");
    }

    @Override
    public void execute() {

        double targetAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
        targetAngularVelocity *= -1;

        m_drive.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY() * 0.75, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getLeftX() * 0.75, OIConstants.kDriveDeadband),
            targetAngularVelocity,
            true,
            true);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setLEDMode_ForceOff("limelight");
    }
    
}
