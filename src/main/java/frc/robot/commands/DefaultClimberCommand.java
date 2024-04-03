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
        double leftVal = MathUtil.applyDeadband(m_controller.getLeftY() * 0.75, OIConstants.kDriveDeadband);
        double rightVal = MathUtil.applyDeadband(m_controller.getRightY() * 0.75, OIConstants.kDriveDeadband);

        /* if the switch isn't pressed, climb freely
         * else if the switch IS pressed, but you want to go up, you can do that
         * else (meaning switch is pressed and you are still trying to go down), don't
         */

        // if (!m_climbersubsystem.isLeftClimberSwitchPressed()) {
        //     m_climbersubsystem.spinLeftMotor(leftVal);
        // } else if (m_climbersubsystem.isLeftClimberSwitchPressed() && leftVal < 0) {
        //     m_climbersubsystem.spinLeftMotor(leftVal);
        // } else {
        //     m_climbersubsystem.stopLeftMotor();
        // }

        // if (!m_climbersubsystem.isRightClimberSwitchPressed()) {
        //     m_climbersubsystem.spinRightMotor(rightVal);
        // } else if (m_climbersubsystem.isRightClimberSwitchPressed() && rightVal < 0) {
        //     m_climbersubsystem.spinLeftMotor(rightVal);
        // } else {
        //     m_climbersubsystem.stopRightMotor();
        // }

        m_climbersubsystem.spinLeftMotor(leftVal);
        m_climbersubsystem.spinRightMotor(rightVal);
        
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
