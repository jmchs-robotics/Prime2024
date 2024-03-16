package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command {
    private final XboxController m_controller;
    private final ClimberSubsystem m_climbersubsystem;

    public DefaultClimberCommand(ClimberSubsystem climbersubsystem, XboxController controller) {
        m_controller = controller;
        m_climbersubsystem = climbersubsystem;
        addRequirements(climbersubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_climbersubsystem.spinLeftMotor(MathUtil.applyDeadband(m_controller.getLeftY() * 0.75, OIConstants.kDriveDeadband));
        m_climbersubsystem.spinRightMotor(MathUtil.applyDeadband(m_controller.getRightY() * 0.75, OIConstants.kDriveDeadband));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climbersubsystem.stopBothMotors();
    }
}
