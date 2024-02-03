// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.commands.autonomous.AutoPaths;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

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

  
  JoystickButton driveA = new JoystickButton(driveStick, XboxController.Button.kA.value);
  JoystickButton subA = new JoystickButton(subStick, XboxController.Button.kA.value);
  JoystickButton subB = new JoystickButton(subStick, XboxController.Button.kB.value);
  JoystickButton subX = new JoystickButton(subStick, XboxController.Button.kX.value);
  JoystickButton subY = new JoystickButton(subStick, XboxController.Button.kY.value);

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
      new IndexInwards(m_intake)
    );

    subY.whileTrue(
      new IndexOutwards(m_intake)
    );
    
    driveA.whileTrue(
        new ShootForward(m_shooter)
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
      case "ra2":
        autoCommand = p.redAmpSide2Note();
        break;
      case "ra3":
        autoCommand = p.redAmpSide3Note();
        break;
      case "rso2":
        autoCommand = p.redSourceSide2Note();
        break;
      case "rso3":
        autoCommand = p.redSourceSide3Note();
        break;
      case "rst2":
        autoCommand = p.redStageSide2Note();
        break;
      case "ba2":
        autoCommand = p.blueAmpSide2Note();
        break;
      case "ba3":
        autoCommand = p.blueAmpSide3Note();
        break;
      case "bso2":
        autoCommand = p.blueSourceSide2Note();
        break;
      case "bso3":
        autoCommand = p.blueSourceSide3Note();
        break;
      case "bst2":
        autoCommand = p.blueStageSide2Note();
        break;      
    }

    return autoCommand;
  }
}
