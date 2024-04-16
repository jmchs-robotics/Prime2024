package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveRobotRelative extends Command {

    private DriveSubsystem m_drive;
    private XboxController m_controller;

    public DriveRobotRelative(DriveSubsystem drive, XboxController controller) {

        m_drive = drive;
        m_controller = controller;

        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_drive.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY() * 0.9, OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_controller.getLeftX() * 0.9, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getRightX() * 0.9, OIConstants.kDriveDeadband),
            false, 
            true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
    
}
