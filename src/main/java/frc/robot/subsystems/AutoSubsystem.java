package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ShootForwardTurbo;

public class AutoSubsystem extends SubsystemBase {

    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
    private Command autoCommand = Commands.runOnce(() -> {});

    GenericEntry autoEntry;

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;

    public AutoSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake) {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto Tab");
        autoEntry = table.getTopic("Auto Path Sequence").getGenericEntry();

        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
    }

    @Override
    public void periodic() {
        validateAndCreatePaths();
    }

    public void setUpAutoTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add("Auto Path Sequence", "").withSize(3, 1).withPosition(0, 0);
        autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
    }

    public void drawPaths() {
        clearField();
        for (int i = 0; i < trajectories.size(); i++) {
            PathPlannerTrajectory pathTraj = trajectories.get(i);
            List<State> states = convertStatesToStates(pathTraj.getStates());
            Trajectory displayTrajectory = new Trajectory(states);

            RobotContainer.field.getObject("traj" + i).setTrajectory(displayTrajectory);
        }
    }

    public void clearField() {
        for (int i = 0; i < 100; i++) {
            FieldObject2d obj = RobotContainer.field.getObject("traj" + i);
            obj.setTrajectory(new Trajectory());
        }
    }

    public void clearAll() {
        trajectories.clear();
        clearField();
    }

    public List<State> convertStatesToStates(List<PathPlannerTrajectory.State> ppStates) {
        ArrayList<State> wpiStates = new ArrayList<State>();
        
        for (int i = 0; i < ppStates.size(); i++) {
            PathPlannerTrajectory.State currentState = ppStates.get(i);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                wpiStates.add(new State(
                    currentState.timeSeconds,
                    currentState.velocityMps,
                    currentState.accelerationMpsSq,
                    new Pose2d(16.54175-currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
                    currentState.curvatureRadPerMeter
                ));
            } else {
                wpiStates.add(new State(
                    currentState.timeSeconds,
                    currentState.velocityMps,
                    currentState.accelerationMpsSq,
                    new Pose2d(16.54175-currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
                    currentState.curvatureRadPerMeter
                ));
            }
        }

        return wpiStates;
    }

    public void builPathSequenceOdometry(String autoString) {

        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        trajectories.clear();

        if (autoString.length() == 0) {
            autoCommand = new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5);
            return;
        }
    }
    
}
