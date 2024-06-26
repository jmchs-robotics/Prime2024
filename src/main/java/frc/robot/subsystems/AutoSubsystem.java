package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeInwards;
import frc.robot.commands.ShootForwardTurbo;

public class AutoSubsystem extends SubsystemBase {

    private ArrayList<PathPlannerTrajectory> trajectories = new ArrayList<PathPlannerTrajectory>();
    private Command autoCommand = Commands.runOnce(() -> {});
    private String feedback = "Enter Auto Path Sequence";

    GenericEntry autoEntry;

    private ShooterSubsystem m_shooterSubsystem;
    private IntakeSubsystem m_intakeSubsystem;
    private DriveSubsystem m_driveSubsystem;

    private char[] NOTE_SPOTS = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'};
    private char[] SHOOT_SPOTS = {'1', 'X', '3'};

    public AutoSubsystem(ShooterSubsystem shooter, IntakeSubsystem intake, DriveSubsystem drive) {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto Tab");
        autoEntry = table.getTopic("Auto Path Sequence").getGenericEntry();

        m_shooterSubsystem = shooter;
        m_intakeSubsystem = intake;
        m_driveSubsystem = drive;
    }

    @Override
    public void periodic() {
        // validateAndCreatePaths();
    }

    public Command getAutoCommand() {
        validateAndCreatePaths();
        return autoCommand;
    }

    public void setUpAutoTab() {
        ShuffleboardTab autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add("Auto Path Sequence", "").withSize(3, 1).withPosition(0, 0);
        autoTab.add(RobotContainer.field).withSize(6, 4).withPosition(3, 0);
        autoTab.addString("Feedback", () -> feedback).withSize(3, 1).withPosition(0, 1);

        NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Auto Tab");

        ntTable.addListener(
            "Auto Path Sequence",
            EnumSet.of(Kind.kValueAll),
            (table, key, event) -> {
                validateAndCreatePaths();
            }
        );
    }

    public void setFeedback(String val) {
        feedback = val;
    }

    public String getFeedback() {
        return feedback;
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
                    new Pose2d(currentState.positionMeters.getX(), currentState.positionMeters.getY(), currentState.heading),
                    currentState.curvatureRadPerMeter
                ));
            }
        }

        return wpiStates;
    }

    public void validateAndCreatePaths() {
        String autoString = autoEntry.getString("");

        buildPathSequenceOdometry(autoString);
        drawPaths();
    }

    public void buildPathSequenceOdometry(String autoString) {

        SequentialCommandGroup finalPath = new SequentialCommandGroup();

        trajectories.clear();

        if (autoString.length() <= 1) {
            autoCommand = new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5);
            setFeedback("Default Path (Shoot and Sit)");
            return;
        }

        // TODO: Put this back in once we do PID tuning
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            if (autoString.charAt(0) == '1') {
                m_driveSubsystem.resetOdometry(new Pose2d(16.54175-0.97, 6.92, Rotation2d.fromDegrees(-60)));
                finalPath.addCommands(new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5));
            } else if (autoString.charAt(0) == '2') {
                m_driveSubsystem.resetOdometry(new Pose2d(16.54175-1.36, 5.54, Rotation2d.fromDegrees(0)));
            } else if (autoString.charAt(0) == '3') {
                m_driveSubsystem.resetOdometry(new Pose2d(16.54175-0.97, 4.13, Rotation2d.fromDegrees(60)));
                finalPath.addCommands(new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5));
            }
        } else {
            if (autoString.charAt(0) == '1') {
                m_driveSubsystem.resetOdometry(new Pose2d(0.97, 6.92, Rotation2d.fromDegrees(-120)));
                finalPath.addCommands(new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5));
            } else if (autoString.charAt(0) == '2') {
                m_driveSubsystem.resetOdometry(new Pose2d(1.36, 5.54, Rotation2d.fromDegrees(180)));
            } else if (autoString.charAt(0) == '3') {
                m_driveSubsystem.resetOdometry(new Pose2d(0.97, 4.13, Rotation2d.fromDegrees(120)));
                finalPath.addCommands(new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5));
            }
        }        

        ParallelRaceGroup segment = new ParallelRaceGroup();
        for (int i = 0; i < autoString.length() - 1; i++) {
            segment = new ParallelRaceGroup();
            char currentPoint = autoString.charAt(i);
            char nextPoint = autoString.charAt(i + 1);

            try {
                if (nextPoint != currentPoint) {
                    PathPlannerPath path = PathPlannerPath.fromPathFile("" + currentPoint + "-" + nextPoint);
                    trajectories.add(
                        path.getTrajectory(
                            new ChassisSpeeds(), 
                            path.getPreviewStartingHolonomicPose().getRotation()
                        )
                    );
                    Command cmd = Commands.sequence(AutoBuilder.followPath(path), new WaitCommand(0.25));
                    segment = new ParallelRaceGroup(cmd);
                }
            } catch (Exception e) {
                setFeedback("Couldn't Find Path File");
                autoCommand = Commands.runOnce(() -> {});
                return;
            }

            // TODO: Put back in once PID tuning is done
            if (indexOfAutoChar(NOTE_SPOTS, nextPoint) != -1) {
                segment.addCommands(new IntakeInwards(m_intakeSubsystem));
            }            

            finalPath.addCommands(segment);

            // TODO: Put back in once PID tuning is done
            if (indexOfAutoChar(SHOOT_SPOTS, nextPoint) != -1) {
                finalPath.addCommands(new ShootForwardTurbo(m_shooterSubsystem, m_intakeSubsystem).withTimeout(1.5));
            }
        }

        autoCommand = finalPath;
        setFeedback("Created Path Sequence");
    }

    public int indexOfAutoChar(char[] arr, char val) {
        for (int i = 0; i < arr.length; i++) {
            if (arr[i] == val) {
                return i;
            }
        }
        return -1;
    }
    
}
