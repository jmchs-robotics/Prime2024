package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;
    private final TalonFX indexMotor;

    public Intake() {

        rightIntakeMotor = new TalonFX(IntakeConstants.rightIntakeID);
        leftIntakeMotor = new TalonFX(IntakeConstants.leftIntakeID);
        indexMotor = new TalonFX(IntakeConstants.indexID);

        rightIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        leftIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        indexMotor.setNeutralMode(NeutralModeValue.Coast);

        rightIntakeMotor.setInverted(false);
        leftIntakeMotor.setInverted(false);
        indexMotor.setInverted(true);

    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void setIntake(double speed) {
        rightIntakeMotor.set(speed);
        leftIntakeMotor.set(speed);
    }

    public void stopIntake() {
       rightIntakeMotor.stopMotor();
       leftIntakeMotor.stopMotor();
    }

    public void setIndex(double speed) {
        indexMotor.set(speed);
    }

    public void stopIndex() {
        indexMotor.stopMotor();
    }
    
}
