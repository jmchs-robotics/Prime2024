package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.AmpConstants;

public class AmpSubsystem extends SubsystemBase {

    private final TalonFX ampMotor;

    public AmpSubsystem() {

        ampMotor = new TalonFX(AmpConstants.ampMotorID);
        ampMotor.setNeutralMode(NeutralModeValue.Brake);
        ampMotor.setInverted(true);

    }

    public void setAmpMotor(double speed) {
        ampMotor.set(speed);
    }

    public void stopAmpMotor() {
        ampMotor.stopMotor();
    }
    
}
