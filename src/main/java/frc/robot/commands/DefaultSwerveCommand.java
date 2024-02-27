package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.OIConstants;

public class DefaultSwerveCommand extends Command {

    private DriveSubsystem m_subsystem;
    private XboxController m_stick;

    public DefaultSwerveCommand(DriveSubsystem subsystem, XboxController stick) {

        m_subsystem = subsystem;
        m_stick = stick;

        addRequirements(subsystem);

    }

    @Override
    public void initialize() {}

    @Override   
    public void execute() {

        m_subsystem.drive(
            -MathUtil.applyDeadband(m_stick.getLeftY() * 0.75, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_stick.getLeftX() * 0.75, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_stick.getRightX() * 0.75, OIConstants.kDriveDeadband),
            true,
            true
        );

        SmartDashboard.putNumber("Robot Pose Angle", m_subsystem.getHeading());

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopDriveMotors();
    }

}
