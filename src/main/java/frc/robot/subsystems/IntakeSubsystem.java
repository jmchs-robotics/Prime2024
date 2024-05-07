package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rightIntakeMotor;
    private final TalonFX leftIntakeMotor;
    private final TalonFX indexMotor;

    private final DigitalInput beamBreak;
    private boolean useBeamBreak = true;

    // private final CANSparkMax indexMotor;

    public IntakeSubsystem() {
        rightIntakeMotor = new TalonFX(IntakeConstants.rightIntakeID);
        leftIntakeMotor = new TalonFX(IntakeConstants.leftIntakeID);
        indexMotor = new TalonFX(IntakeConstants.indexID);
        // indexMotor = new CANSparkMax(IntakeConstants.indexID, MotorType.kBrushless);

        rightIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        leftIntakeMotor.setNeutralMode(NeutralModeValue.Coast);
        indexMotor.setNeutralMode(NeutralModeValue.Coast);
        // indexMotor.setIdleMode(IdleMode.kCoast);

        rightIntakeMotor.setInverted(false);
        leftIntakeMotor.setInverted(false);
        indexMotor.setInverted(true);

        // indexMotor.burnFlash();

        beamBreak = new DigitalInput(IntakeConstants.beamBreakPort);
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
