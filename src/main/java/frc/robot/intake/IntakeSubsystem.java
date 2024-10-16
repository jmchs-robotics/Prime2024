package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;
    private final TalonFX indexMotor;

    private final DigitalInput beamBreak;
    private boolean useBeamBreak = true;

    public IntakeSubsystem() {
        rightIntakeMotor = new TalonFX(3);
        leftIntakeMotor = new TalonFX(4);
        indexMotor = new TalonFX(7);

        rightIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        leftIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        indexMotor.setNeutralMode(NeutralModeValue.Coast);

        rightIntakeMotor.setInverted(false);
        leftIntakeMotor.setInverted(false);
        indexMotor.setInverted(true);

        beamBreak = new DigitalInput(0);
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

    public boolean isBeamBreakTripped() {
        return !beamBreak.get();
    }

    public void toggleBeamBreak() {
        if (useBeamBreak) {
            useBeamBreak = false;
        } else {
            useBeamBreak = true;
        }
    }

    public boolean isUsingBeamBreak() {
        return useBeamBreak;
    }
}
