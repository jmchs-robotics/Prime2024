package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;

/**
 * Command to turn the robot (set the robot's pose) to the desired angle
 * Constructor takes robot's drivetrain and desired angle
 */
public class SetPoseAngle extends Command {
    private static final double ANGLE_CHECK_TIME = 0.1;
    private static final double TARGET_ANGLE_BUFFER = 5.0;

    private final DriveSubsystem drivetrain;
    private final double targetAngle;
    private final PIDController angleController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    /**

     * turn the robot to the targetAngle
     Field oriented.
     * @param drivetrain (DriveSubsystem)
     * @param targetAngle angle to turn to, in degrees (double) Positive is CCW, negative is CW.
     */
    public SetPoseAngle(DriveSubsystem drivetrain, double targetAngle) {
        this.drivetrain = drivetrain;

        /*if (targetAngle < 0)
            targetAngle += 360;            
        this.targetAngle = targetAngle;
        */
        this.targetAngle = (targetAngle + 360) % 360; // normalize to range [0,360)
        angleController = new PIDController(0.02, 0, 0);
        angleController.enableContinuousInput(0, 360);
        angleController.reset();
        /*
        angleController = new PIDController(0.03, 0, 0.075, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) {
            }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return drivetrain.getGyroAngle();
            }
        }, output -> {
            for (int i = 0; i < 4; i++)
                drivetrain.getSwerveModule(i).setTargetSpeed(output);
        });
        if (Robot.PRACTICE_BOT)
            angleController.setP(0.025);
        
        angleController.setInputRange(0, 360);
        angleController.setOutputRange(-0.5, 0.5);
        angleController.setContinuous(true);
*/
        angleController.setSetpoint(targetAngle);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        double a = -(DriveConstants.kWheelBase / DriveConstants.kTrackWidth);
        double b = (DriveConstants.kWheelBase / DriveConstants.kTrackWidth);
        double c = -(DriveConstants.kTrackWidth / DriveConstants.kWheelBase);
        double d = (DriveConstants.kTrackWidth / DriveConstants.kWheelBase);

        double[] angles = new double[]{
                Math.atan2(b, c),
                Math.atan2(b, d),
                Math.atan2(a, d),
                Math.atan2(a, c)
        };

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(Math.toDegrees(angles[i]));
        }

        //angleController.enable();
    }

    @Override
    public void execute() {
        double output = angleController.calculate(drivetrain.getHeading());
        for (int i = 0; i < 4; i++)
            drivetrain.getSwerveModule(i).setTargetSpeed(output);
    }

    @Override
    public boolean isFinished() {
        double currentAngle = drivetrain.getHeading();
        double currentError = currentAngle - targetAngle;

        boolean inTargetBuffer = Math.abs(currentError) < TARGET_ANGLE_BUFFER
                | 360 - Math.abs(currentError) < TARGET_ANGLE_BUFFER;

        if (inTargetBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasElapsed(ANGLE_CHECK_TIME);
    }

    @Override
    public void end( boolean isInterrupted) {
        // angleController.disable();
        drivetrain.drive(0, 0, 0, false, false);
    }
}