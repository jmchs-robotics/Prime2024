package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    // private final TalonFX rightShooterMotor;
    private final CANSparkFlex rightShooterMotor;
    // private final TalonFX leftShooterMotor;
    private final CANSparkFlex leftShooterMotor;

    public ShooterSubsystem() {
        // rightShooterMotor = new TalonFX(ShooterConstants.rightShooterID);
        // addChild("rightShooterTalon", rightShooterMotor);
        // rightShooterMotor.setInverted(true);

        rightShooterMotor = new CANSparkFlex(ShooterConstants.rightShooterID, MotorType.kBrushless);
        rightShooterMotor.setInverted(true);

        // leftShooterMotor = new TalonFX(ShooterConstants.leftShooterID);
        // addChild("leftShooterTalon", leftShooterMotor);
        // leftShooterMotor.setInverted(false);

        leftShooterMotor = new CANSparkFlex(ShooterConstants.leftShooterID, MotorType.kBrushless);
        leftShooterMotor.setInverted(false);

        // rightShooterMotor.setNeutralMode(NeutralModeValue.Brake);
        rightShooterMotor.setIdleMode(IdleMode.kCoast);
        // leftShooterMotor.setNeutralMode(NeutralModeValue.Coast);
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
