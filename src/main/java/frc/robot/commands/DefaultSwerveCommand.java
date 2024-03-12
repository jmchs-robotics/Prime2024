package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;

public class DefaultSwerveCommand extends Command {
    private DriveSubsystem m_drivesubsystem;
    private XboxController m_controller;

    public DefaultSwerveCommand(DriveSubsystem drivesubsystem, XboxController controller) {
        m_drivesubsystem = drivesubsystem;
        m_controller = controller;
        addRequirements(drivesubsystem);
    }

    @Override
    public void initialize() {}

    @Override   
    public void execute() {
        m_drivesubsystem.drive(
            -MathUtil.applyDeadband(m_controller.getLeftY() * 0.9, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getLeftX() * 0.9, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_controller.getRightX() * 0.9, OIConstants.kDriveDeadband),
            true,
            true
        );
        SmartDashboard.putNumber("Robot Angle", m_drivesubsystem.getHeading());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_drivesubsystem.stopDriveMotors();
    }
}
