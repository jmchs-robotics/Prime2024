package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor;
    private final TalonFX indexMotor;

    public Intake() {

        intakeMotor = new TalonFX(3);
        indexMotor = new TalonFX(4);

    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void intakePiece(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntakeMotor() {
        intakeMotor.stopMotor();
    }
    
    public void indexPiece(double speed) {
        indexMotor.set(speed);
    }

    public void stopindexMotor() {
        indexMotor.stopMotor();
    }

    public void spinBothMotors(double speed) {
        intakePiece(speed);
        indexPiece(speed);
    }

    public void stopBothMotors() {
        stopIntakeMotor();
        stopindexMotor();
    }
    
}
