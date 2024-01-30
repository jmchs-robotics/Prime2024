package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command {

    private final XboxController m_stick;
    private final ClimberSubsystem m_subsystem;

    public DefaultClimberCommand(ClimberSubsystem subsystem, XboxController stick) {

        m_stick = stick;

        m_subsystem = subsystem;
        addRequirements(subsystem);

    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        m_subsystem.spinLeftMotor(MathUtil.applyDeadband(m_stick.getLeftY(), OIConstants.kDriveDeadband));
        m_subsystem.spinRightMotor(MathUtil.applyDeadband(m_stick.getRightY(), OIConstants.kDriveDeadband));

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopBothMotors();
    }
    
}
