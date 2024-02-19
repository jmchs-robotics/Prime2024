package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;

    public Intake() {

        rightIntakeMotor = new TalonFX(IntakeConstants.rightIntakeID);
        leftIntakeMotor = new TalonFX(IntakeConstants.leftIntakeID);

        rightIntakeMotor.setNeutralMode(NeutralModeValue.Brake);
        leftIntakeMotor.setNeutralMode(NeutralModeValue.Brake);

        rightIntakeMotor.setInverted(false);
        leftIntakeMotor.setInverted(false);

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
    
}
