package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class AutoSubsystem extends SubsystemBase {

    public AutoSubsystem() {}

    public void setUpAutoTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
    }
    
}
