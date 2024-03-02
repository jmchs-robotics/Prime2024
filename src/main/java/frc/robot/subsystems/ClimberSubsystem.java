package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

    private final TalonFX rightClimberMotor;
    private final TalonFX leftClimberMotor;

    public ClimberSubsystem() {
        rightClimberMotor = new TalonFX(ClimberConstants.rightClimberID);
        leftClimberMotor = new TalonFX(ClimberConstants.leftClimberID);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void spinRightMotor(double speed) {
        rightClimberMotor.set(speed);
    }

    public void stopRightMotor() {
        rightClimberMotor.stopMotor();
    }
    
    public void spinLeftMotor(double speed) {
        leftClimberMotor.set(speed);
    }

    public void stopLeftMotor() {
        leftClimberMotor.stopMotor();
    }

    public void spinBothMotors(double speed) {
        spinRightMotor(speed);
        spinLeftMotor(speed);
    }

    public void stopBothMotors() {
        stopRightMotor();
        stopLeftMotor();
    }
}
