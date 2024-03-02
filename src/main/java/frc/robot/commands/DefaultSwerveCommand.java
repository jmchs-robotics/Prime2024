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
            -MathUtil.applyDeadband(m_stick.getLeftY() * 0.7, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_stick.getLeftX() * 0.7, OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_stick.getRightX() * 0.7, OIConstants.kDriveDeadband),
            true,
            false
        );
        
        SmartDashboard.putNumber("Robot Gyro Angle", m_subsystem.getHeading());
        SmartDashboard.putNumber("Robot Gyro Angle Rate", m_subsystem.getTurnRate());

        SmartDashboard.putNumber("Front Left Drive Encoder Value", m_subsystem.m_frontLeft.m_drivingEncoder.getPosition());
        SmartDashboard.putNumber("Front Right Drive Encoder Value", m_subsystem.m_frontRight.m_drivingEncoder.getPosition());
        SmartDashboard.putNumber("Back Left Drive Encoder Value", m_subsystem.m_rearLeft.m_drivingEncoder.getPosition());
        SmartDashboard.putNumber("Back Right Drive Encoder Value", m_subsystem.m_rearRight.m_drivingEncoder.getPosition());

        SmartDashboard.putNumber("Front Left Turning Encoder Value", m_subsystem.m_frontLeft.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Front Right Turning Encoder Value", m_subsystem.m_frontRight.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Back Left Turning Encoder Value", m_subsystem.m_rearLeft.m_turningEncoder.getPosition());
        SmartDashboard.putNumber("Back Right Turning Encoder Value", m_subsystem.m_rearRight.m_turningEncoder.getPosition());

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
