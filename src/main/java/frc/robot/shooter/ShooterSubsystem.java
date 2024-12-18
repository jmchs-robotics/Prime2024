package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX topShooterMotor;
    private final TalonFX bottomShooterMotor;

    public ShooterSubsystem() {

        topShooterMotor = new TalonFX(1);
        topShooterMotor.setInverted(true);

        bottomShooterMotor = new TalonFX(2);
        bottomShooterMotor.setInverted(false);

        topShooterMotor.setNeutralMode(NeutralModeValue.Coast);
        bottomShooterMotor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void setTopShooterMotor(double speed) {
        topShooterMotor.set(speed);
    }

    public void stopTopShooterMotor() {
        topShooterMotor.stopMotor();
    }

    public void setBottomShooterMotor(double speed) {
        bottomShooterMotor.set(speed);
    }

    public void stopBottomShooterMotor() {
        bottomShooterMotor.stopMotor();
    }

    public void setBothShooterMotors(double speed) {
        topShooterMotor.set(speed);
        bottomShooterMotor.set(speed);
    }

    public void stopBothShooterMotors() {
        topShooterMotor.stopMotor();
        bottomShooterMotor.stopMotor();
    }
}
