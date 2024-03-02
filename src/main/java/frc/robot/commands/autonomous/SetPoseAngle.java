package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

/**
 * Command to turn the robot (set the robot's pose) to the desired angle
 * Constructor takes robot's drivetrain and desired angle
 */
public class SetPoseAngle extends Command {
    // private static final double ANGLE_CHECK_TIME = 0.1;
    // private static final double TARGET_ANGLE_BUFFER = 5.0;

    private final DriveSubsystem drivetrain;
    private final double targetAngle;
    private final boolean turnRight; // true is right, false is left
    // private final PIDController angleController;
    // private final Timer finishTimer = new Timer();
    // private boolean isTimerStarted = false;

    /**

     * turn the robot to the targetAngle
     Field oriented.
     * @param drivetrain (DriveSubsystem)
     * @param targetAngle angle to turn to, in degrees (double) in range (-180, 180] CW positive
     */
    public SetPoseAngle(DriveSubsystem drivetrain, double targetAngle) {
//         this.drivetrain = drivetrain;

//         /*if (targetAngle < 0)
//             targetAngle += 360;            
//         this.targetAngle = targetAngle;
//         */
//         this.targetAngle = (targetAngle + 360) % 360; // normalize to range [0,360)
//         angleController = new PIDController(0.02, 0, 0);
//         angleController.enableContinuousInput(0, 360);
//         angleController.reset();
//         /*
//         angleController = new PIDController(0.03, 0, 0.075, new PIDSource() {
//             @Override
//             public void setPIDSourceType(PIDSourceType pidSource) {
//             }

//             @Override
//             public PIDSourceType getPIDSourceType() {
//                 return PIDSourceType.kDisplacement;
//             }

//             @Override
//             public double pidGet() {
//                 return drivetrain.getGyroAngle();
//             }
//         }, output -> {
//             for (int i = 0; i < 4; i++)
//                 drivetrain.getSwerveModule(i).setTargetSpeed(output);
//         });
//         if (Robot.PRACTICE_BOT)
//             angleController.setP(0.025);
        
//         angleController.setInputRange(0, 360);
//         angleController.setOutputRange(-0.5, 0.5);
//         angleController.setContinuous(true);
// */
//         angleController.setSetpoint(targetAngle);

//         addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.targetAngle = targetAngle;

        double angleDifference = targetAngle - drivetrain.getHeading();

        if (angleDifference < -180) {
            angleDifference += 360;
        } else if (angleDifference > 180) {
            angleDifference -= 360;
        }

        if (angleDifference < 0) {
            turnRight = false;
        } else {
            turnRight = true;
        }
    }

    @Override
    public void initialize() {
        // finishTimer.stop();
        // finishTimer.reset();
        // isTimerStarted = false;

        // double a = -(DriveConstants.kWheelBase / DriveConstants.kTrackWidth);
        // double b = (DriveConstants.kWheelBase / DriveConstants.kTrackWidth);
        // double c = -(DriveConstants.kTrackWidth / DriveConstants.kWheelBase);
        // double d = (DriveConstants.kTrackWidth / DriveConstants.kWheelBase);

        // double[] angles = new double[]{
        //         Math.atan2(b, c),
        //         Math.atan2(b, d),
        //         Math.atan2(a, d),
        //         Math.atan2(a, c)
        // };

        // for (int i = 0; i < 4; i++) {
        //     drivetrain.getSwerveModule(i).setTargetAngle(Math.toDegrees(angles[i]));
        // }

        //angleController.enable();

        SwerveModuleState[] turningWheelStates = new SwerveModuleState[4];

        turningWheelStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        turningWheelStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        turningWheelStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        turningWheelStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        drivetrain.setModuleStates(turningWheelStates);

    }

    @Override
    public void execute() {
        // double output = angleController.calculate(drivetrain.getHeading());
        // for (int i = 0; i < 4; i++)
        //     drivetrain.getSwerveModule(i).setTargetSpeed(output);

        // Remember CW positive (in this case)

        // SwerveModuleState[] turningWheelStates = new SwerveModuleState[4];

        // if (turnRight) {
        //     turningWheelStates[0] = new SwerveModuleState(-1, Rotation2d.fromDegrees(45));
        //     turningWheelStates[1] = new SwerveModuleState(1, Rotation2d.fromDegrees(-45));
        //     turningWheelStates[2] = new SwerveModuleState(1, Rotation2d.fromDegrees(-45));
        //     turningWheelStates[3] = new SwerveModuleState(-1, Rotation2d.fromDegrees(45));
        // } else {
        //     turningWheelStates[0] = new SwerveModuleState(1, Rotation2d.fromDegrees(-45));
        //     turningWheelStates[1] = new SwerveModuleState(-1, Rotation2d.fromDegrees(45));
        //     turningWheelStates[2] = new SwerveModuleState(-1, Rotation2d.fromDegrees(45));
        //     turningWheelStates[3] = new SwerveModuleState(1, Rotation2d.fromDegrees(-45));
        // }

        // drivetrain.setModuleStates(turningWheelStates);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = drivetrain.getHeading();
        double currentError = currentAngle - targetAngle;

        if (turnRight) {
            if (targetAngle < 0) {
                if (currentAngle > 0) {
                    return false;
                } else if (currentAngle >= targetAngle) {
                    return true;
                } else {
                    return false;
                }
            } else {
                if (currentAngle >= targetAngle) {
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            if (targetAngle > 0) {
                if (currentAngle < 0) {
                    return false;
                } else if (currentAngle <= targetAngle) {
                    return true;
                } else {
                    return false;
                }
            } else {
                if (currentAngle <= targetAngle) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        // boolean inTargetBuffer = Math.abs(currentError) < TARGET_ANGLE_BUFFER
        //         | 360 - Math.abs(currentError) < TARGET_ANGLE_BUFFER;

        // if (inTargetBuffer) {
        //     if (!isTimerStarted) {
        //         finishTimer.start();
        //         isTimerStarted = true;
        //     }
        // } else {
        //     finishTimer.stop();
        //     finishTimer.reset();
        //     isTimerStarted = false;
        // }

        // return finishTimer.hasElapsed(ANGLE_CHECK_TIME);
    }

    @Override
    public void end( boolean isInterrupted) {
        SmartDashboard.putString("HAHA", "It dont work");
        // angleController.disable();

        // shouldn't need to do any fancy setModuleState stuff
        // because zero drive automatically sets it to 0
        // drivetrain.drive(0, 0, 0, false, false);
    }
}