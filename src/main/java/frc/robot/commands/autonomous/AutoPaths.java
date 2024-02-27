package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
import frc.robot.commands.IntakeOutwards;
import frc.robot.commands.ShootForSpeaker;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.autonomous.*;

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

    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    
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
                new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(6, 0.5, Rotation2d.fromDegrees(0))),            
            config);

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

        // return swerveControllerCommand.andThen(() -> m_drive.drive(0, 0, 0, false, false));
        return new SetPoseAngle(m_drive, 90)
            .andThen(new IntakeInwards(m_intake)).withTimeout(1);

        // return new SequentialCommandGroup(
        //     new InstantCommand(m_drive::setDrivePIDToSlow, m_drive),
        //     new WaitCommand(w),
        //     new DriveForDist(m_drive, 36)
        // );

        // return new SequentialCommandGroup(
        //     new DriveForDistRewrite(36, 0, m_drive),
        //     new WaitCommand(w),
        //     new DriveForDistRewrite(0, 36, m_drive),
        //     new WaitCommand(w),
        //     new DriveForDistRewrite(-36, 0, m_drive),
        //     new WaitCommand(w),
        //     new DriveForDistRewrite(0, -36, m_drive),
        //     new WaitCommand(w)
        // );

    }

    public Command center2NoteBase() {

        Trajectory goOut = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), 0, Rotation2d.fromDegrees(0))),
            config);

        Trajectory goBackIn = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goOutCommand = new SwerveControllerCommand(
            goOut,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand goBackInCommand = new SwerveControllerCommand(
            goBackIn,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(goOut.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            ),
            new WaitCommand(w),
            new ParallelRaceGroup(
                goOutCommand,
                new IntakeInwards(m_intake)
            ).andThen(() -> m_drive.resetOdometry(goBackIn.getInitialPose())),
            new WaitCommand(w),
            goBackInCommand,           
            new WaitCommand(w),
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            )
        );
    }

    public Command red3NoteAmp_blue3NoteSource() {

        Trajectory goFor3Out = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.dist3NoteAngled), Units.inchesToMeters(AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), Units.inchesToMeters(AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0))), 
            config);

        Trajectory goFor3In = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), Units.inchesToMeters(AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.dist3NoteAngled), Units.inchesToMeters(AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goFor3OutCommand = new SwerveControllerCommand(
            goFor3Out,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand goFor3InCommand = new SwerveControllerCommand(
            goFor3In,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        return new SequentialCommandGroup(
            center2NoteBase().andThen(() -> m_drive.resetOdometry(goFor3Out.getInitialPose())),
            new WaitCommand(w),
            new ParallelRaceGroup(
                goFor3OutCommand,
                new IntakeInwards(m_intake)
            ).andThen(() -> m_drive.resetOdometry(goFor3In.getInitialPose())),
            new WaitCommand(w),
            goFor3InCommand,
            new WaitCommand(w),
            new ShootForSpeaker(m_shooter),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            )
        );
    }

    public Command red3NoteSource_blue3NoteAmp() {

        Trajectory goFor3Out = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.dist3NoteAngled), Units.inchesToMeters(-AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), Units.inchesToMeters(-AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0))), 
            config);

        Trajectory goFor3In = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(Units.inchesToMeters(AutoConstants.robotToCenterNote), Units.inchesToMeters(-AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(Units.inchesToMeters(AutoConstants.dist3NoteAngled), Units.inchesToMeters(-AutoConstants.distBetweenNotes), Rotation2d.fromDegrees(0)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goFor3OutCommand = new SwerveControllerCommand(
            goFor3Out,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand goFor3InCommand = new SwerveControllerCommand(
            goFor3In,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        return new SequentialCommandGroup(
            center2NoteBase().andThen(() -> m_drive.resetOdometry(goFor3Out.getInitialPose())),
            new WaitCommand(w),
            new ParallelRaceGroup(
                goFor3OutCommand,
                new IntakeInwards(m_intake)
            ).andThen(() -> m_drive.resetOdometry(goFor3In.getInitialPose())),
            new WaitCommand(w),
            goFor3InCommand,
            new WaitCommand(w),
            new ShootForSpeaker(m_shooter),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            )
        );
    }

    public Command red2NoteAmp_blue2NoteSource() {
        
        Trajectory goOut = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0, Units.inchesToMeters(-38.58), Rotation2d.fromDegrees(90)),
                new Pose2d(Units.inchesToMeters(-36.35), Units.inchesToMeters(-70.68), Rotation2d.fromDegrees(180))),
            config);

        Trajectory goBackIn = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(Units.inchesToMeters(-36.35), Units.inchesToMeters(-70.68), Rotation2d.fromDegrees(180)),
                new Pose2d(0, Units.inchesToMeters(-38.58), Rotation2d.fromDegrees(90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goOutCommand = new SwerveControllerCommand(
            goOut,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand goBackInCommand = new SwerveControllerCommand(
            goBackIn,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(goOut.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            ),
            new WaitCommand(w),
            new ParallelRaceGroup(
                goOutCommand,
                new IntakeInwards(m_intake)
            ).andThen(() -> m_drive.resetOdometry(goBackIn.getInitialPose())),
            new WaitCommand(w),
            goBackInCommand,           
            new WaitCommand(w),
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            )
        );
    }

    public Command red2NoteSource_blue2NoteAmp() {
        
        Trajectory goOut = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                new Pose2d(0, Units.inchesToMeters(38.58), Rotation2d.fromDegrees(-90)),
                new Pose2d(Units.inchesToMeters(-36.35), Units.inchesToMeters(70.68), Rotation2d.fromDegrees(-180))),
            config);

        Trajectory goBackIn = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(Units.inchesToMeters(-36.35), Units.inchesToMeters(70.68), Rotation2d.fromDegrees(-180)),
                new Pose2d(0, Units.inchesToMeters(38.58), Rotation2d.fromDegrees(-90)),
                new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
            config);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand goOutCommand = new SwerveControllerCommand(
            goOut,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        SwerveControllerCommand goBackInCommand = new SwerveControllerCommand(
            goBackIn,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);

        m_drive.resetOdometry(goOut.getInitialPose());

        return new SequentialCommandGroup(
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            ),
            new WaitCommand(w),
            new ParallelRaceGroup(
                goOutCommand,
                new IntakeInwards(m_intake)
            ).andThen(() -> m_drive.resetOdometry(goBackIn.getInitialPose())),
            new WaitCommand(w),
            goBackInCommand,           
            new WaitCommand(w),
            new ShootForSpeaker(m_shooter).withTimeout(0.5),
            new ParallelCommandGroup(
                new IntakeInwards(m_intake).withTimeout(1),
                new ShootForSpeaker(m_shooter).withTimeout(1)
            )
        );
    }

}