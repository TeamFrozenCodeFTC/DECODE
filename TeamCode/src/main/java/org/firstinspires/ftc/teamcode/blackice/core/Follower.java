package org.firstinspires.ftc.teamcode.blackice.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blackice.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.blackice.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;
import org.firstinspires.ftc.teamcode.blackice.localizers.Localizer;
import org.firstinspires.ftc.teamcode.blackice.localizers.LocalizerConfig;

import java.util.function.DoubleSupplier;

public class Follower {
    /**
     * Responsible for turning the robot and making sure it is facing the correct
     * direction.
     */
    public final PDController headingController;
    
    /**
     * Responsible holding a given pose and giving translational power for the robot to
     * stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    public final PredictiveBrakingController positionalController;

    public final Drivetrain drivetrain;
    public final Localizer localizer;
    
    public PoseTolerance poseTolerance;
    public MotionTolerance motionTolerance;
    
    boolean isBraking = false;
    Vector brakingVector;
    public double deltaTime;
    double lastTime = System.nanoTime();
    DoubleSupplier voltageSupplier;

    public Follower(PDController headingController,
                    PredictiveBrakingController positionalController,
                    DrivetrainConfig drivetrain,
                    LocalizerConfig localizer,
                    HardwareMap hardwareMap,
                    PoseTolerance poseTolerance,
                    MotionTolerance motionTolerance) {
        this.headingController = headingController;
        this.positionalController = positionalController;
        this.drivetrain = drivetrain.build(hardwareMap);
        this.localizer = localizer.build(hardwareMap);
        this.drivetrain.zeroPowerFloatMode();
        this.poseTolerance = poseTolerance;
        this.motionTolerance = motionTolerance;
        this.voltageSupplier = () -> {
            double minV = Double.POSITIVE_INFINITY;
            for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                double v = vs.getVoltage();
                if (v > 0) minV = Math.min(minV, v);
            }
            return Math.max(9.0, Math.min(14.5, minV));
        };
    }
    
    public Vector getVelocity() {
        return localizer.getVelocity();
    }
    
    public Vector getPosition() {
        return localizer.getPose().getPosition();
    }
    
    public double getHeading() {
        return localizer.getPose().getHeading();
    }
    
    
    // hold pose, follow path, follow composite path

    public void reset() {
        headingController.reset();
        isBraking = false;
        brakingVector = null;
    }
    
    public boolean isStoppedAt(Pose pose) {
        return poseTolerance.atPose(pose, localizer.getPose())
            && motionTolerance.isStopped(localizer.getVelocity(),
                     localizer.getAngularVelocity());
    }
    
    public boolean isWithinBraking(Pose pose) {
        return isWithinBraking(pose.getPosition());
    }
    
    public boolean isWithinBraking(Vector position) {
        return computeHoldPower(position).dot(position.minus(localizer.getPose().getPosition())) < 1;
    }
    
    public double getVoltage() {
        return voltageSupplier.getAsDouble();
    }
    
    public Pose getCurrentPose() {
        return localizer.getPose();
    }

    /**
     * Instructs the drivetrain to follow a vector relative to the field.
     */
    public void followFieldVector(Vector fieldVector, double turnPower) {
        Vector robotVector = localizer.toRobotRelativeVector(fieldVector)
            .map(localizer.toRobotRelativeVector(localizer.getVelocity()),
                 this::clampReversePower);
        drivetrain.followVector(robotVector, turnPower);
    }
    
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) {
            return power;
        }
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -0.2);
        } else {
            clampedPower = Math.min(power, 0.2);
        }
        return clampedPower;
    }
    
    /**
     * Returns true if the robot is within braking distance of the target pose.
     */
    public void holdPose(Pose pose) {
        holdPose(pose, 1);
    }
    
    /**
     * Returns true if the robot is within braking distance of the target pose.
     */
    public boolean holdPose(Pose pose, double maxPower) {
        Vector holdPower = computeHoldPower(pose.getPosition());
        double powerMag = holdPower.computeMagnitude();
        if (powerMag > maxPower) {
            holdPower = holdPower.times(maxPower / powerMag);
        }

        double turnPower = Math.min(maxPower,
            computeHeadingCorrectionPower(pose.getHeading()));

        followFieldVector(holdPower, turnPower);
        
        return powerMag < 1;
    }
    
    public Vector computeHoldPower(Vector position) {
        Vector error =
            position.minus(localizer.getPose().getPosition());
        
        return error.map(localizer.getVelocity(),
                         positionalController::computeOutput);
    }
    
    /**
     * Amount of power needed to brake to a stop from the current velocity.
     */
    public Vector computeBrakingPower() {
        return new Vector(0,0)
            .map(localizer.getVelocity(), positionalController::computeOutput);
    }

    public double computeHeadingCorrectionPower(double targetHeading) {
        double headingError =
            AngleUnit.RADIANS.normalize(targetHeading - localizer.getPose().getHeading());

        return headingController.computeOutput(headingError, deltaTime);
    }
    
    private double calculateDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) * 1e-9;
        lastTime = currentTime;
        return deltaTime;
    }
    
    public void update() {
        deltaTime = calculateDeltaTime();
        localizer.update(deltaTime);
    }
    
    public void setCurrentHeading(double headingDegrees) {
        setCurrentPose(new Pose(localizer.getPose().getPosition().getX(),
                                localizer.getPose().getPosition().getY(), headingDegrees));
    }
    
    public void setCurrentPose(Pose pose) {
        localizer.setCurrentPose(pose.getPosition().getX(), pose.getPosition().getY(),
                        pose.getHeading());
        localizer.update(deltaTime);
        teleOpTarget = pose;
    }
    
    public enum DriveState {
        MANUAL,
        DECELERATING,
        HOLDING
    }
    
    public static class Config {
        public double stopVelocityThreshold = 0.02;
        public boolean holdPositionOnStop = true;
        public boolean autoBrakeEnabled = true;
        public boolean headingLockEnabled = true;
    }
    
    private DriveState driveState = DriveState.MANUAL;
    private final Config config = new Config();
    
    private Pose teleOpTarget = new Pose(0, 0, 0);
    private Double lockedHeading = null;
    
    public void setLockedHeading(Double headingRad) {
        this.lockedHeading = headingRad;
    }
    
    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
        boolean hasInput = forward != 0 || lateral != 0 || turn != 0;
        
        updateDriveState(hasInput);
        
        switch (driveState) {
            case MANUAL:
                driveManual(forward, lateral, turn);
                break;
            case DECELERATING:
                driveDecelerating();
                break;
            case HOLDING:
                driveHolding();
                break;
        }
    }
    
    private void updateDriveState(boolean hasInput) {
        if (hasInput || !config.holdPositionOnStop) {
            driveState = DriveState.MANUAL;
            return;
        }
        
        if (driveState == DriveState.MANUAL) {
            driveState = DriveState.DECELERATING;
            return;
        }
        
        if (driveState == DriveState.DECELERATING) {
            if (localizer.getVelocity().computeMagnitude() < config.stopVelocityThreshold) {
                teleOpTarget = localizer.getPose();
                driveState = DriveState.HOLDING;
            }
        }
    }
    
    private void driveManual(double forward, double lateral, double turn) {
        if (lockedHeading != null && config.headingLockEnabled) {
            // locked heading is radians
            turn = computeHeadingCorrectionPower(lockedHeading);
        }
        
        drivetrain.followVector(
            localizer.toRobotRelativeVector(new Vector(forward, lateral)),
            turn
        );
    }
    
    private void driveDecelerating() {
//        if (config.autoBrakeEnabled) {
//            drivetrain.followVector(computeBrakingPower(),
//                                    computeHeadingCorrectionPower(
//                                        teleOpTarget.getHeading()));
//        } else {
//            drivetrain.zeroPower();
//        }
        drivetrain.zeroPower();
    }
    
    private void driveHolding() {
        if (lockedHeading != null) {
            teleOpTarget =
                new Pose(teleOpTarget.getPosition(), Math.toDegrees(lockedHeading));
        }
        holdPose(teleOpTarget);
    }
    
    public void robotCentricDrive(double forward, double lateral, double turn) {
        drivetrain.followVector(new Vector(forward, lateral), turn);
    }
}
