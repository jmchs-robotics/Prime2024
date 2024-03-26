// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public static final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public static final ClimberSubsystem m_climber = new ClimberSubsystem();
  public static final IntakeSubsystem m_intake = new IntakeSubsystem();
  public static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public static final AmpSubsystem m_amp = new AmpSubsystem();
  public static final AutoSubsystem m_auto = new AutoSubsystem(m_shooter, m_intake);

  public static final Field2d field = new Field2d();

  // The driver's controller
  XboxController driveStick = new XboxController(OIConstants.kDriverControllerPort);
  XboxController subStick = new XboxController(OIConstants.kOperatorControllerPort);
  
  JoystickButton driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);
  JoystickButton driveStart = new JoystickButton(driveStick, XboxController.Button.kStart.value);
  JoystickButton driveLB = new JoystickButton(driveStick, XboxController.Button.kLeftBumper.value);
  JoystickButton driveRB = new JoystickButton(driveStick, XboxController.Button.kRightBumper.value);
  JoystickButton subA = new JoystickButton(subStick, XboxController.Button.kA.value);
  JoystickButton subB = new JoystickButton(subStick, XboxController.Button.kB.value);
  JoystickButton subX = new JoystickButton(subStick, XboxController.Button.kX.value);
  JoystickButton subY = new JoystickButton(subStick, XboxController.Button.kY.value);
  JoystickButton subLB = new JoystickButton(subStick, XboxController.Button.kLeftBumper.value);
  JoystickButton subRB = new JoystickButton(subStick, XboxController.Button.kRightBumper.value);
  JoystickButton subStart = new JoystickButton(subStick, XboxController.Button.kStart.value);
  JoystickButton subBack = new JoystickButton(subStick, XboxController.Button.kBack.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(new DefaultSwerveCommand(m_robotDrive, driveStick));
    m_climber.setDefaultCommand(new DefaultClimberCommand(m_climber, subStick));
    m_intake.setDefaultCommand(new DefaultIntakeCommand(m_intake));
    m_shooter.setDefaultCommand(new DefaultShooterCommand(m_shooter));

    NamedCommands.registerCommand("Intake Note", new IntakeInwards(m_intake));
    NamedCommands.registerCommand("Shoot Note", new ShootForwardTurbo(m_shooter, m_intake));
    NamedCommands.registerCommand("Reverse Shooter", new ReverseShooter(m_shooter));
    NamedCommands.registerCommand("Reverse Intake", new IntakeOutwards(m_intake));
    NamedCommands.registerCommand("Limelight Aim", new LimelightAiming(m_robotDrive, driveStick));

    m_robotDrive.setUpDriveTab();
    m_auto.setUpAutoTab();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    subA.whileTrue(
      new IntakeInwards(m_intake)
    );

    subB.whileTrue(
      new IntakeOutwards(m_intake)
    );
    
    subX.whileTrue(
        new ShootForwardTurbo(m_shooter, m_intake)
    );

    subRB.and(subX).whileTrue(
      new ShootForAmp(m_shooter)
    );

    subY.whileTrue(
      new ReverseShooter(m_shooter)
    );

    subStart.whileTrue(
      new FlipAmpForward(m_amp)
    );

    subBack.whileTrue(
      new FlipAmpBackward(m_amp)
    );

    driveStart.onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );

    driveLB.whileTrue(
      new LimelightAiming(m_robotDrive, driveStick)
    );

    driveRB.whileTrue(
      new ClimbBothDown(m_climber)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_auto.getAutoCommand();

    // Command autoCommand = null;

    // switch(a) {
    //   case "topSide":
    //     autoCommand = m_robotDrive.getAuto("TopSideAuto");
    //     break;
    //   case "bottomSide":
    //     autoCommand = m_robotDrive.getAuto("BottomSideAuto");
    //     break;
    //   case "centerSide":
    //     autoCommand = m_robotDrive.getAuto("CenterSideAuto");
    //     break;
    //   case "centerSide3Bottom":
    //     autoCommand = new SequentialCommandGroup(
    //       m_robotDrive.getAuto("CenterSideAuto"),
    //       m_robotDrive.getAuto("CenterSideAuto3NoteBottom"),
    //       new WaitCommand(0.25),
    //       new InstantCommand(m_shooter::stopBothShooterMotors),
    //       new InstantCommand(m_intake::stopIntake),
    //       new InstantCommand(m_intake::stopIndex),
    //       new RunCommand(() -> m_robotDrive.drive(0, 0, 0, false, false), m_robotDrive)
    //     );
    //     break;
    // }

    // return autoCommand;
  }
}
