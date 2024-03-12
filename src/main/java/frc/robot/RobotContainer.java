// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutoPaths;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ClimberSubsystem m_climber = new ClimberSubsystem();
  public final Intake m_intake = new Intake();
  public final Shooter m_shooter = new Shooter();

  // The driver's controller
  XboxController driveStick = new XboxController(OIConstants.kDriverControllerPort);
  XboxController subStick = new XboxController(OIConstants.kOperatorControllerPort);
  Joystick planeStick = new Joystick(OIConstants.kDriverControllerPort);
  
  JoystickButton driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);
  JoystickButton driveStart = new JoystickButton(driveStick, XboxController.Button.kStart.value);
  JoystickButton driveLB = new JoystickButton(driveStick, XboxController.Button.kLeftBumper.value);
  JoystickButton subA = new JoystickButton(subStick, XboxController.Button.kA.value);
  JoystickButton subB = new JoystickButton(subStick, XboxController.Button.kB.value);
  JoystickButton subX = new JoystickButton(subStick, XboxController.Button.kX.value);
  JoystickButton subY = new JoystickButton(subStick, XboxController.Button.kY.value);
  JoystickButton subLB = new JoystickButton(subStick, XboxController.Button.kLeftBumper.value);
  JoystickButton subRB = new JoystickButton(subStick, XboxController.Button.kRightBumper.value);

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
    // new JoystickButton(driveStick, Button.kR1.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

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

    driveStart.onTrue(
      new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive)
    );

    driveA.whileTrue(
      new LimelightAiming(m_robotDrive)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String a) {
    
    AutoPaths p = new AutoPaths(m_robotDrive, m_shooter, m_intake, m_climber);

    Command autoCommand = null;

    switch(a) {
      case "c2":
        autoCommand = p.center2NoteBase();
        break;
      case "ra3bs3":
        autoCommand = p.red3NoteAmp_blue3NoteSource();
        break;
      case "rs3ba3":
        autoCommand = p.red3NoteSource_blue3NoteAmp();
        break;
      case "ra2bs2":
        autoCommand = p.red2NoteAmp_blue2NoteSource();
        break;
      case "rs2ba2":
        autoCommand = p.red2NoteSource_blue2NoteAmp();
        break;
      case "topSide":
        autoCommand = m_robotDrive.getAuto("TopSideAuto");
        break;
      case "bottomSide":
        autoCommand = m_robotDrive.getAuto("BottomSideAuto");
        break;
      case "centerSide":
        autoCommand = m_robotDrive.getAuto("CenterSideAuto");
    }

    return autoCommand;
  }
}
