package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final TalonFX rightShooterMotor;
    // private final TalonFX leftShooterMotor;

    public Shooter() {

        rightShooterMotor = new TalonFX(ShooterConstants.rightShooterID);
        addChild("rightShooterTalon", rightShooterMotor);
        rightShooterMotor.setInverted(true);
        // leftShooterMotor = new TalonFX(ShooterConstants.leftShooterID);

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
    
}
