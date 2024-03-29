package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final TalonFX rightShooterMotor;
    private final TalonFX leftShooterMotor;

    public Shooter() {

        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterID);
        addChild("rightShooterTalon", rightShooterMotor);
        rightShooterMotor.setInverted(true);

        leftShooterMotor = new TalonFX(ShooterConstants.leftShooterID);
        addChild("leftShooterTalon", leftShooterMotor);
        leftShooterMotor.setInverted(false);

        rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        leftShooterMotor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

    public void setRightShooterMotor(double speed) {
        rightShooterMotor.set(speed);
    }

    public void stopRightShooterMotor() {
        rightShooterMotor.stopMotor();
    }

    public void setLeftShooterMotor(double speed) {
        leftShooterMotor.set(speed);
    }

    public void stopLeftShooterMotor() {
        leftShooterMotor.stopMotor();
    }

    public void setBothShooterMotors(double speed) {
        rightShooterMotor.set(speed);
        leftShooterMotor.set(speed);
    }

    public void stopBothShooterMotors() {
        rightShooterMotor.stopMotor();
        leftShooterMotor.stopMotor();
    }
    
}
