package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public Command test() {

        Trajectory test = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                // new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
                // new Pose2d(2, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(3, 0, Rotation2d.fromDegrees(0))),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            test,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(test.getInitialPose());

        return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, false, false));

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

    public Command redAmpSide3Note() {

        Trajectory redAmp3NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory redAmp3NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);
        
        Trajectory redAmp3NoteTrajectory3rdNoteForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(57.0 / 2))),
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(57.0), new Rotation2d(Units.degreesToRadians(41.9))),
            config);

        Trajectory redAmp3NoteTrajectory3rdNoteBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(57.0), new Rotation2d(Units.degreesToRadians(41.9))),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(57.0 / 2))),
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

        Trajectory redAmp3NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand redAmp3NoteCommandForward = new SwerveControllerCommand(
            redAmp3NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand redAmp3NoteCommandBackward = new SwerveControllerCommand(
            redAmp3NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redAmp3NoteCommand3rdNoteForward = new SwerveControllerCommand(
            redAmp3NoteTrajectory3rdNoteForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redAmp3NoteCommand3rdNoteBackward = new SwerveControllerCommand(
            redAmp3NoteTrajectory3rdNoteBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand redAmp3NoteCommandGTFO = new SwerveControllerCommand(
            redAmp3NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(redAmp3NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                redAmp3NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp3NoteTrajectoryBackward.getInitialPose())),
            redAmp3NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp3NoteTrajectory3rdNoteForward.getInitialPose())),
            new ParallelCommandGroup(
                redAmp3NoteCommand3rdNoteForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp3NoteTrajectory3rdNoteBackward.getInitialPose())),
            redAmp3NoteCommand3rdNoteBackward,
            new WaitCommand(w),
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redAmp3NoteTrajectoryGTFO.getInitialPose())),
            redAmp3NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );
        
    }

    public Command redSourceSide2Note() {
         Trajectory redSource2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(-6.85))),
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(-13.7), new Rotation2d(Units.degreesToRadians(21.2))), config);

        Trajectory redSource2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(-13.7), new Rotation2d(21.2)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(-6.85))),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory redSource2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(147.4), Units.inchesToMeters(-103.2))),
            new Pose2d(Units.inchesToMeters(297.4), Units.inchesToMeters(-103.2), new Rotation2d(Units.degreesToRadians(30))),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand redSource2NoteCommandForward = new SwerveControllerCommand(
            redSource2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redSource2NoteCommandBackward = new SwerveControllerCommand(
            redSource2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redSource2NoteCommandGTFO = new SwerveControllerCommand(
            redSource2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(redSource2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                redSource2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redSource2NoteTrajectoryBackward.getInitialPose())),
            redSource2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redSource2NoteTrajectoryGTFO.getInitialPose())),
            redSource2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }

    public Command redSourceSide3Note() {

        Trajectory redStage3NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory redStage3NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);
        
        Trajectory redStage3NoteTrajectory3rdNoteForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(-57.0 / 2))),
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(-57.0), new Rotation2d(Units.degreesToRadians(-41.9))),
            config);

        Trajectory redStage3NoteTrajectory3rdNoteBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(-57.0), new Rotation2d(Units.degreesToRadians(-41.9))),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(-57.0 / 2))),
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

        Trajectory redStage3NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(-46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(-46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand redStage3NoteCommandForward = new SwerveControllerCommand(
            redStage3NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand redStage3NoteCommandBackward = new SwerveControllerCommand(
            redStage3NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redStage3NoteCommand3rdNoteForward = new SwerveControllerCommand(
            redStage3NoteTrajectory3rdNoteForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand redStage3NoteCommand3rdNoteBackward = new SwerveControllerCommand(
            redStage3NoteTrajectory3rdNoteBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand redStage3NoteCommandGTFO = new SwerveControllerCommand(
            redStage3NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(redStage3NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                redStage3NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage3NoteTrajectoryBackward.getInitialPose())),
            redStage3NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage3NoteTrajectory3rdNoteForward.getInitialPose())),
            new ParallelCommandGroup(
                redStage3NoteCommand3rdNoteForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage3NoteTrajectory3rdNoteBackward.getInitialPose())),
            redStage3NoteCommand3rdNoteBackward,
            new WaitCommand(w),
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(redStage3NoteTrajectoryGTFO.getInitialPose())),
            redStage3NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }

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

    public Command blueAmpSide2Note() {

        Trajectory blueAmp2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(-6.85))),
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(-13.7), new Rotation2d(Units.degreesToRadians(21.2))), config);

        Trajectory blueAmp2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(-13.7), new Rotation2d(21.2)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(-6.85))),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory blueAmp2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(307.7 / 2), Units.inchesToMeters(-37.8 / 2))),
            new Pose2d(Units.inchesToMeters(307.7), Units.inchesToMeters(37.8), new Rotation2d(Units.degreesToRadians(-30))),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand blueAmp2NoteCommandForward = new SwerveControllerCommand(
            blueAmp2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueAmp2NoteCommandBackward = new SwerveControllerCommand(
            blueAmp2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueAmp2NoteCommandGTFO = new SwerveControllerCommand(
            blueAmp2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(blueAmp2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                blueAmp2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp2NoteTrajectoryBackward.getInitialPose())),
            blueAmp2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp2NoteTrajectoryGTFO.getInitialPose())),
            blueAmp2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }

    public Command blueAmpSide3Note() {

        Trajectory blueAmp3NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory blueAmp3NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);
        
        Trajectory blueAmp3NoteTrajectory3rdNoteForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(-57.0 / 2))),
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(-57.0), new Rotation2d(Units.degreesToRadians(-41.9))),
            config);

        Trajectory blueAmp3NoteTrajectory3rdNoteBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(-57.0), new Rotation2d(Units.degreesToRadians(-41.9))),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(-57.0 / 2))),
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

        Trajectory blueAmp3NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(-46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(-46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand blueAmp3NoteCommandForward = new SwerveControllerCommand(
            blueAmp3NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand blueAmp3NoteCommandBackward = new SwerveControllerCommand(
            blueAmp3NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueAmp3NoteCommand3rdNoteForward = new SwerveControllerCommand(
            blueAmp3NoteTrajectory3rdNoteForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueAmp3NoteCommand3rdNoteBackward = new SwerveControllerCommand(
            blueAmp3NoteTrajectory3rdNoteBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand blueAmp3NoteCommandGTFO = new SwerveControllerCommand(
            blueAmp3NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(blueAmp3NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                blueAmp3NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp3NoteTrajectoryBackward.getInitialPose())),
            blueAmp3NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp3NoteTrajectory3rdNoteForward.getInitialPose())),
            new ParallelCommandGroup(
                blueAmp3NoteCommand3rdNoteForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp3NoteTrajectory3rdNoteBackward.getInitialPose())),
            blueAmp3NoteCommand3rdNoteBackward,
            new WaitCommand(w),
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueAmp3NoteTrajectoryGTFO.getInitialPose())),
            blueAmp3NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }

    public Command blueSourceSide2Note() {

        Trajectory blueSource2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(6.85))),
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(13.7), new Rotation2d(Units.degreesToRadians(-21.2))), config);

        Trajectory blueSource2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(88.4), Units.inchesToMeters(13.7), new Rotation2d(-21.2)), 
            List.of(new Translation2d(Units.inchesToMeters(44.2), Units.inchesToMeters(6.85))),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory blueSource2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(147.4), Units.inchesToMeters(103.2))),
            new Pose2d(Units.inchesToMeters(297.4), Units.inchesToMeters(103.2), new Rotation2d(Units.degreesToRadians(-30))),
            config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand blueSource2NoteCommandForward = new SwerveControllerCommand(
            blueSource2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueSource2NoteCommandBackward = new SwerveControllerCommand(
            blueSource2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueSource2NoteCommandGTFO = new SwerveControllerCommand(
            blueSource2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(blueSource2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                blueSource2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource2NoteTrajectoryBackward.getInitialPose())),
            blueSource2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource2NoteTrajectoryGTFO.getInitialPose())),
            blueSource2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }  
    
    public Command blueSourceSide3Note() {

        Trajectory blueSource3NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory blueSource3NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);
        
        Trajectory blueSource3NoteTrajectory3rdNoteForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(57.0 / 2))),
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(57.0), new Rotation2d(Units.degreesToRadians(41.9))),
            config);

        Trajectory blueSource3NoteTrajectory3rdNoteBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(63.5), Units.inchesToMeters(57.0), new Rotation2d(Units.degreesToRadians(41.9))),
            List.of(new Translation2d(Units.inchesToMeters(63.5 / 2), Units.inchesToMeters(57.0 / 2))),
            new Pose2d(0, 0, new Rotation2d(0)),
            config);

        Trajectory blueSource3NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand blueSource3NoteCommandForward = new SwerveControllerCommand(
            blueSource3NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand blueSource3NoteCommandBackward = new SwerveControllerCommand(
            blueSource3NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueSource3NoteCommand3rdNoteForward = new SwerveControllerCommand(
            blueSource3NoteTrajectory3rdNoteForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueSource3NoteCommand3rdNoteBackward = new SwerveControllerCommand(
            blueSource3NoteTrajectory3rdNoteBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        SwerveControllerCommand blueSource3NoteCommandGTFO = new SwerveControllerCommand(
            blueSource3NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(blueSource3NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                blueSource3NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource3NoteTrajectoryBackward.getInitialPose())),
            blueSource3NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource3NoteTrajectory3rdNoteForward.getInitialPose())),
            new ParallelCommandGroup(
                blueSource3NoteCommand3rdNoteForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource3NoteTrajectory3rdNoteBackward.getInitialPose())),
            blueSource3NoteCommand3rdNoteBackward,
            new WaitCommand(w),
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueSource3NoteTrajectoryGTFO.getInitialPose())),
            blueSource3NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );

    }

    public Command blueStageSide2Note() {

        Trajectory blueStage2NoteTrajectoryForward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), config);

        Trajectory blueStage2NoteTrajectoryBackward = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(64.0), 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(32.0), 0)),
            new Pose2d(0, 0, new Rotation2d(0)), config);

        Trajectory blueStage2NoteTrajectoryGTFO = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)), 
            List.of(new Translation2d(Units.inchesToMeters(87.0), 0),
                new Translation2d(Units.inchesToMeters(87.0 + 96.2), Units.inchesToMeters(-46.9))),
            new Pose2d(Units.inchesToMeters(87.0 + 96.2 + 90), Units.inchesToMeters(-46.9), new Rotation2d(0)), config);

        var thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand blueStage2NoteCommandForward = new SwerveControllerCommand(
            blueStage2NoteTrajectoryForward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueStage2NoteCommandBackward = new SwerveControllerCommand(
            blueStage2NoteTrajectoryBackward,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand blueStage2NoteCommandGTFO = new SwerveControllerCommand(
            blueStage2NoteTrajectoryGTFO,
            m_drive::getPose, 
            DriveConstants.kDriveKinematics, 
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0), 
            thetaController,
            m_drive::setModuleStates,
            m_drive);
        
        m_drive.resetOdometry(blueStage2NoteTrajectoryForward.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w),
            new ParallelCommandGroup(
                blueStage2NoteCommandForward,
                new IntakeInwards(m_intake)
            ),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueStage2NoteTrajectoryBackward.getInitialPose())),
            blueStage2NoteCommandBackward,
            new ShootForward(m_shooter).withTimeout(1),
            new WaitCommand(w).andThen(() -> m_drive.resetOdometry(blueStage2NoteTrajectoryGTFO.getInitialPose())),
            blueStage2NoteCommandGTFO.andThen(() -> m_drive.drive(0, 0, 0, false, false))
        );
        
    }
}
