package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbBothDown extends Command {
    private final ClimberSubsystem m_climbersubsystem;

    public ClimbBothDown(ClimberSubsystem climbersubsystem) {
        m_climbersubsystem = climbersubsystem;
        addRequirements(climbersubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (!m_climbersubsystem.isLeftClimberSwitchPressed()) {
            m_climbersubsystem.spinLeftMotor(0.75);
        } else {
            m_climbersubsystem.stopLeftMotor();
        }
        if (!m_climbersubsystem.isRightClimberSwitchPressed()) {
            m_climbersubsystem.spinRightMotor(0.75);
        } else {
            m_climbersubsystem.stopRightMotor();
        }
    }

    @Override
    public boolean isFinished() {
        return (m_climbersubsystem.isLeftClimberSwitchPressed() && m_climbersubsystem.isRightClimberSwitchPressed());
    }

    @Override
    public void end(boolean interrupted) {
        m_climbersubsystem.stopBothMotors();
    }
}
