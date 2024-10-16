// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.drive.DriveCommands;
import frc.robot.drive.DriveSubsystem;
import frc.robot.intake.IntakeCommands;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.shooter.ShooterCommands;
import frc.robot.shooter.ShooterSubsystem;
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
  public final DriveSubsystem m_drive = new DriveSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final Auto m_auto = new Auto(m_shooter, m_intake, m_drive);

  public static final Field2d field = new Field2d();

  // The driver's controller
  XboxController driveStick = new XboxController(0);
  XboxController subStick = new XboxController(1);
  
  JoystickButton driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);
  JoystickButton driveStart = new JoystickButton(driveStick, XboxController.Button.kStart.value);
  JoystickButton driveBack = new JoystickButton(driveStick, XboxController.Button.kBack.value);
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

    NamedCommands.registerCommand("Intake Note", IntakeCommands.intakeInwards(m_intake));
    NamedCommands.registerCommand("Shoot Note", ShooterCommands.shootSpeaker(m_shooter, m_intake));
    NamedCommands.registerCommand("Reverse Shooter", ShooterCommands.reverseShooter(m_shooter));

    setUpDriveTab();
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

    m_drive.setDefaultCommand(
      DriveCommands.driveFieldRelative(
        m_drive, 
        driveStick.getLeftY(), 
        driveStick.getLeftX(), 
        driveStick.getRightX())
    );

    subA.whileTrue(
      IntakeCommands.intakeInwards(m_intake)
    );

    subB.whileTrue(
      IntakeCommands.intakeOutwards(m_intake)
    );
    
    subX.whileTrue(
        ShooterCommands.shootSpeaker(m_shooter, m_intake)
    );

    subY.whileTrue(
      ShooterCommands.reverseShooter(m_shooter)
    );

    driveStart.onTrue(
      new InstantCommand(() -> m_drive.zeroHeading(), m_drive)
    );

    driveBack.onTrue(
      new InstantCommand(() -> {m_intake.toggleBeamBreak();}, m_intake)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_auto.getAutoCommand();

  }

  public void setUpDriveTab() {
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive Tab");

    driveTab.addBoolean("Robot Ate Note",
      () -> {
        return m_intake.isBeamBreakTripped();
      }).withPosition(0, 0)
      .withSize(2, 2)
      .withWidget(BuiltInWidgets.kBooleanBox);

    driveTab.addDouble("Match Time Remaining",
      () -> {return (int) Timer.getMatchTime();})
      .withPosition(0, 3)
      .withSize(2, 2)
      .withWidget(BuiltInWidgets.kDial)
      .withProperties(Map.of("min", 0, "max", 135));

    driveTab.addBoolean("Is Using BeamBreak",
      () -> {return m_intake.isUsingBeamBreak();})
      .withPosition(9, 0)
      .withSize(1, 1)
      .withWidget(BuiltInWidgets.kBooleanBox);
  }
}
