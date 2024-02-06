package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ShootForward extends Command {

    private Shooter m_subsystem;

    private NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

    private double limelightAngle = 0; // Maybe won't need these, we'll see
    private double limelightHeight = 0; // Maybe won't need these, we'll see

    public ShootForward(Shooter subsystem) {

        m_subsystem = subsystem;
        addRequirements(m_subsystem);
    
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        double tY = m_table.getEntry("ty").getDouble(0.0);
        double tV = m_table.getEntry("tv").getDouble(0.0);

        // Should be minimum of 36 inches
        double distanceError = -tY;

        // Assuming max velocity of note will be 400 in/s
        // Max velocity should be reached when we hug the subwoofer
        // 400 (velocity) = 36 (distanceError) * 11.1
        // 1 (in percent) = 36 * 11.1 * 0.04
        // 1 = distanceError * 0.444
        // 0.444 needs to be negative to equate to a positive shoot velocity
        m_subsystem.setBothShooterMotors(distanceError * -0.444);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopRightShooterMotor();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
    
}
