package frc.robot.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterSubsystem extends SubsystemBase {

    private final CANSparkFlex rightShooterMotor;
    private final CANSparkFlex leftShooterMotor;

    public ShooterSubsystem() {

        rightShooterMotor = new CANSparkFlex(1, MotorType.kBrushless);
        rightShooterMotor.setInverted(true);

        leftShooterMotor = new CANSparkFlex(2, MotorType.kBrushless);
        leftShooterMotor.setInverted(false);

        rightShooterMotor.setIdleMode(IdleMode.kCoast);
        leftShooterMotor.setIdleMode(IdleMode.kCoast);

        rightShooterMotor.setOpenLoopRampRate(0.75);
        leftShooterMotor.setOpenLoopRampRate(0.75);

        rightShooterMotor.burnFlash();
        leftShooterMotor.burnFlash();
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
