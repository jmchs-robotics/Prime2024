package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeInwards;
import frc.robot.commands.ShootForward;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import java.util.List;
import edu.wpi.first.math.util.Units;

public class AutoPaths extends Command {

    DriveSubsystem m_drive;
    Shooter m_shooter;
    Intake m_intake;
    ClimberSubsystem m_climber;

    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);
    
    double w = 0.25;

    public AutoPaths(DriveSubsystem drive, Shooter shooter, Intake intake, ClimberSubsystem climber) {
        m_drive = drive;
        m_shooter = shooter;
        m_intake = intake;
        m_climber = climber;

        addRequirements(m_drive, m_shooter, m_intake, m_climber);
    }

    public Command redAmpSide2Note() {

        Trajectory redAmp2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(6.85))),
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(13.7), new Rotation2d(Units.degreesToRadians(-21.2))), config);

        Trajectory redAmp2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(13.7), new Rotation2d(-21.2)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(6.85))),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory redAmp2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(307.7 / 2), Units.inchesToMeters(37.8 / 2))),
            new Pose2d(Units.inchesToMeters(307.7), Units.inchesToMeters(37.8), new Rotation2d(Units.degreesToRadians(-30))),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand redAmp2NoteCommandForward = new SwerveControllerCommand(
            redAmp2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redAmp2NoteCommandBackward = new SwerveControllerCommand(
            redAmp2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redAmp2NoteCommandGTFO = new SwerveControllerCommand(
            redAmp2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(redAmp2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                redAmp2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp2NoteTrajectoryBackward.getInitialPose())),
            redAmp2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp2NoteTrajectoryGTFO.getInitialPose())),
            redAmp2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }  

    // public Command redAmpSide3Note() {}
    // public Command redSourceSide2Note() {}
    // public Command redSourceSide3Note() {}

    public Command redStageSide2Note() {

        Trajectory redStage2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory redStage2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory redStage2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand redStage2NoteCommandForward = new SwerveControllerCommand(
            redStage2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redStage2NoteCommandBackward = new SwerveControllerCommand(
            redStage2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redStage2NoteCommandGTFO = new SwerveControllerCommand(
            redStage2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(redStage2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                redStage2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage2NoteTrajectoryBackward.getInitialPose())),
            redStage2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage2NoteTrajectoryGTFO.getInitialPose())),
            redStage2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );
        
    }

    // public Command blueAmpSide2Note() {}
    // public Command blueAmpSide3Note() {}
    // public Command blueSourceSide2Note() {}
    // public Command blueSourceSide3Note() {}
    // public Command blueStageSide2Note() {}
    
}
