package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.MAXSwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

public class DriveForDist extends Command {
    private static final double TARGET_DISTANCE_BUFFER = 4; // 2;
    private static final double DISTANCE_CHECK_TIME = 0.25;

    private final DriveSubsystem drivetrain;
    private final double angle;
    private final double distance;
    private final double distRight, distForward;
    private final PIDController angleErrorController;
    private final Timer finishTimer = new Timer();
    private boolean isTimerStarted = false;

    private double initialDrivetrainAngle = 0;
    private double rotationFactor = 0;

    private double rotationMinOutput = -1;
    private double forwardMaxOutput = 1;
    private double strafeMaxOutput = 1;
    private double rotationMaxOutput = 1;

    private BufferedWriter[] encPosLoggers = new BufferedWriter[4];
    private BufferedWriter[] encVelLoggers = new BufferedWriter[4];
    private int iterCount;

    private boolean isGyroSet = false;

    /**
     * Move the robot forward/backwards this many inches. 
     * Field oriented.
     * @param drivetrain 
     * @param distance inches; positive if forwards, negative is backwards.
     */
    public DriveForDist(DriveSubsystem drivetrain, double distance) {
        this(drivetrain, 0, distance);
    }

    /**
     * Move the robot this many inches.  Robot maintains its same pose angle.
     * Field oriented.
     * @param drivetrain
     * @param distRight inches positive is to the right; negative is to the left.
     * @param distForward inches positive is forward; negaitve is backwards.
     */
    public DriveForDist(DriveSubsystem drivetrain, double distRight, double distForward) {
        this.drivetrain = drivetrain;
        this.angle = Math.toDegrees(Math.atan2(distRight, distForward));
        
        this.distRight = distRight; // -distRight;  // 191206 seems like this should not get inverted
        this.distForward = distForward;
        
        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);
        // Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // 191206 this PID isn't working... probably needs more P.  Original is 0.02, 0, 0
        angleErrorController = new PIDController(0.02, 0, 0);
        angleErrorController.enableContinuousInput(0, 360);
        angleErrorController.reset();

        /*
         new PIDController(0.02, 0, 0, new PIDSource() {
            @Override
            public void setPIDSourceType(PIDSourceType pidSource) { }

            @Override
            public PIDSourceType getPIDSourceType() {
                return PIDSourceType.kDisplacement;
            }

            @Override
            public double pidGet() {
                return drivetrain.getGyroAngle();
            }
        }, output -> {
            rotationFactor = output;
        });
        
        angleErrorController.setInputRange(0, 360);
        angleErrorController.setOutputRange(-0.5, 0.5);
        angleErrorController.setContinuous(true);
        */
        addRequirements(drivetrain);
    }

    public DriveForDist(DriveSubsystem drivetrain, double distRight, double distForward, double poseAngle) {
        this.drivetrain = drivetrain;
        this.angle = poseAngle;
        
        this.distRight = distRight; // -distRight;  // 191206 seems like this should not get inverted
        this.distForward = distForward;
        
        this.distance = Math.sqrt(distRight * distRight + distForward * distForward);
        // Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // // 191206 this PID isn't working... probably needs more P.  Original is 0.02, 0, 0
        angleErrorController = new PIDController(0.02, 0, 0);
        angleErrorController.enableContinuousInput(0, 360);
        angleErrorController.reset();

        isGyroSet = true;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        resetPID();
        finishTimer.stop();
        finishTimer.reset();
        isTimerStarted = false;

        if (isGyroSet) {
            initialDrivetrainAngle = angle;
        }
        else {
            initialDrivetrainAngle = drivetrain.getHeading();
        }
        
        angleErrorController.setSetpoint(initialDrivetrainAngle);

        //Calculations done by AngleErrorController are invalid, will be applying in about line 145:
        // angleErrorController.enable();

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(angle + initialDrivetrainAngle); // drivetrain.getGyroAngle());
            drivetrain.getSwerveModule(i).resetEncoders();
            drivetrain.getSwerveModule(i).setTargetDistance(distance);
        }

        iterCount = 0;

    }

    @Override
    public void execute() {
        double forwardFactor = distForward / distance;
        double strafeFactor = -distRight / distance;
        double g = drivetrain.getHeading(); // measurement
        // setting the inital drive angle minus the gyro angle
        double x = (initialDrivetrainAngle - g) % 360;
        if( x > 180) {
            x -= 360;
        }
        else if (x < -180){
            x += 360;
        }
        double rotation =  x  * (0.02); //angleErrorController.calculate(x); //x * (DrivetrainConstants.DFD_ROTATION_kP); 
        
        // rotation = Math.min( -0.5, Math.max( 0.5, rotation));  // clamp

        double[] moduleAngles = drivetrain.calculateSwerveModuleAngles(forwardFactor, strafeFactor, rotation);  // -rotationFactor); // 191206

        for (int i = 0; i < 4; i++) {
            drivetrain.getSwerveModule(i).setTargetAngle(moduleAngles[i]);            
        }
        iterCount++;
        
    }

    /**
     * This Command finishes when all 4 of the the robot's wheels stay within TARGET_DISTANCE_BUFFER (inches)
     * of the desired drive distance for DISTANC_CHECK_TIME
     */
    @Override
    public boolean isFinished() {
        boolean inBuffer = true;
        for (int i = 0; i < 4; i++) {
            // check that all wheels are within the TARGET_DISTANCE_BUFFER of the desired travel distance
            inBuffer &= Math.abs(distance - Math.abs(drivetrain.getSwerveModule(i).getDriveDistance())) < TARGET_DISTANCE_BUFFER;
        }

        if (inBuffer) {
            if (!isTimerStarted) {
                finishTimer.start();
                isTimerStarted = true;
            }
        } else {
            finishTimer.stop();
            finishTimer.reset();
            isTimerStarted = false;
        }

        return finishTimer.hasElapsed(DISTANCE_CHECK_TIME);
    }

    @Override
    public void end(boolean isInterrupted) {
        drivetrain.drive(0, 0, 0, false, false);

        // angleErrorController.disable();
    }

    public void resetPID() {
        angleErrorController.reset();
        angleErrorController.disableContinuousInput();
        angleErrorController.setSetpoint(0);
        angleErrorController.setIntegratorRange(-1.0, 1.0);
        angleErrorController.setTolerance(0.05, Double.POSITIVE_INFINITY);
        // initialDrivetrainAngle = 0;

        rotationMinOutput = -1;
        forwardMaxOutput = 1;
        strafeMaxOutput = 1;
        rotationMaxOutput = 1;
    }
}