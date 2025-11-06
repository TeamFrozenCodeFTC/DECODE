package org.firstinspires.ftc.blackice.core.follower;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.blackice.FollowerConstants;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Drivetrain;
import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.routines.PathRoutineBuilder;
import org.firstinspires.ftc.blackice.util.DashboardUtils;
import org.firstinspires.ftc.blackice.util.Logger;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * Orchestrator
 */
public class Follower extends PathRoutineController {
    private static Follower INSTANCE;
    private static Pose lastOpModePose;
    public final Drivetrain drivetrain;
    public final Pose startingPose;
    private final double pauseWhenVoltageBelow;
    public boolean teleOpIsHolding = false;
    public boolean teleOpIsDecelerating = true;
    public Pose teleOpTarget;
    private PathBehavior defaultPathBehavior;
    private boolean lowVoltage = false;
    private double voltage = 0;
    
    public Follower(
        HardwareMap hardwareMap,
        FollowerConfig config,
        Pose startingPose
    ) {
        super(new DrivePowerController(
                  config.headingPID,
                  config.positionalPID,
                  config.driveVelocityPIDF,
                  config.centripetalFeedforward,
                  config.maxReversalBrakingPower,
                  config.drivetrainConfig.build(hardwareMap),
                  config.naturalDeceleration,
                  config.tunedVoltage,
                  () -> {
                      double minV = Double.POSITIVE_INFINITY;
                      for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                          double v = vs.getVoltage();
                          if (v > 0) minV = Math.min(minV, v);
                      }
                      if (minV == Double.POSITIVE_INFINITY) return config.tunedVoltage;
                      return Math.max(9.0, Math.min(14.5, minV));
                  }
              ), config.localizerConfig.createMotionTracker(hardwareMap)
        );
        this.drivetrain = getDrivetrain();
        INSTANCE = this;
        this.startingPose = startingPose;
        setCurrentPose(startingPose);
        
        this.defaultPathBehavior = config.defaultPathBehavior;
        this.pauseWhenVoltageBelow = config.stopIfVoltageBelow;
        
        DashboardUtils.addPublicFieldsRecursive(this, "follower");
    }
    
    public Follower(HardwareMap hardwareMap, Pose startingPose) {
        this(hardwareMap, FollowerConstants.defaultFollowerConfig, startingPose);
    }
    
    /**
     * Initializes the follower at Pose(0,0,0)
     */
    public Follower(HardwareMap hardwareMap) {
        this(hardwareMap, new Pose(0, 0, 0));
    }
    
    public static Follower getInstance() {
        return INSTANCE;
    }
    
    public static Pose getLastOpModePose() {
        return lastOpModePose;
    }
    
    public void addDefaultPathBehavior(PathBehavior behavior) {
        defaultPathBehavior = defaultPathBehavior.mergeWith(behavior);
    }
    
    public void holdPose(Pose pose) {
        drivePowerController.holdPose(pose, getMotionState());
    }

    public boolean isAt(Pose pose, PoseTolerance tolerance) {
        return tolerance.isPoseWithinTolerance(pose, getCurrentPose());
    }
    
    public boolean isStoppedAt(Pose pose, PoseTolerance tolerance,
                               VelocityTolerance velocityTolerance) {
        return isAt(pose, tolerance) && velocityTolerance.isVelocityWithinTolerance(getMotionState());
    }
    
    public boolean isStoppedAt(Pose pose) {
        //h=.5, pos=0.25
        return isAt(pose, new PoseTolerance(0.5, 3))
            && (new VelocityTolerance(0.25, 5))
         .isVelocityWithinTolerance(getMotionState());
    }
    
    public Pose getCurrentPose() {
        return getMotionState().pose;
    }
    
    /**
     * Returns a new {@link PathRoutineBuilder} with the default path behavior of this
     * follower.
     * <p>
     * This is the way to create paths.
     */
    public PathRoutineBuilder pathRoutineBuilder() {
        return new PathRoutineBuilder(this::getCurrentPose)
            .withStartPose(startingPose)
            .withDefaultBehavior(defaultPathBehavior);
    }
    
    public PathRoutineBuilder pathRoutineBuilder(Pose startPose) {
        return pathRoutineBuilder()
            .withStartPose(startPose);
    }
    
    private void logVoltage() {
        double voltage = drivePowerController.getVoltage();
        
        if (voltage < pauseWhenVoltageBelow && !lowVoltage) {
            pause();
            lowVoltage = true;
            Logger.warn("LOW VOLTAGE:  " + voltage + ". PAUSING FOLLOWER");
        } else if (voltage < 10 && lowVoltage) {
            resume();
            lowVoltage = false;
            Logger.warn("VOLTAGE BACK UP TO " + voltage + ". RESUMING FOLLOWER");
        } else {
            Logger.verbose("Voltage: ", voltage);
        }
    }
    
    public void update() {
        logVoltage();
        super.update();
    }
    
    public double getVoltage() {
        return drivePowerController.getVoltage();
    }

    /**
     * Initializes the robot for tele-op mode, using the position from the end of the
     * autonomous period.
     */
    public void initTeleOp() {
        setCurrentPose(lastOpModePose);
        drivetrain.zeroPowerBrakeMode();
    }
    
    public void savePoseForTeleOp() {
        lastOpModePose = new Pose(
            getMotionState().position,
            Math.toDegrees(getMotionState().heading)
        );
    }
    
    /**
     * Run a basic field-centric tele-op.
     * <pre><code>
     * follower.fieldCentricTeleOpDrive(
     *     gamepad1.left_stick_x,
     *     -gamepad1.left_stick_y
     *     -gamepad1.right_stick_x
     * );
     * </code></pre>
     */
    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
        MotionState motionState = getMotionState();
        
        boolean noInput = forward == 0 && lateral == 0 && turn == 0;
        
        if (noInput && !teleOpIsHolding && !teleOpIsDecelerating) {
            teleOpIsDecelerating = true;
        }
        
        if (motionState.speed < 0.25 && teleOpIsDecelerating) {
            teleOpIsDecelerating = false;
            teleOpIsHolding = true;
            teleOpTarget = motionState.pose;
        }
        
        if (!noInput || teleOpIsDecelerating) {
            teleOpIsHolding = false;
            drivetrain.followVector(motionState.makeRobotRelative(new Vector(forward,lateral)), turn);
        } else {
            holdPose(teleOpTarget);
        }
    }
    
    public void goalCentricTeleOpDrive(double forward, double lateral,
                                        double lockedHeading) {
        MotionState motionState = getMotionState();

        boolean noInput = forward == 0 && lateral == 0;
        
        if (noInput && !teleOpIsHolding && !teleOpIsDecelerating) {
            teleOpIsDecelerating = true;
        }
        
        if (motionState.speed < 0.25 && teleOpIsDecelerating) {
            teleOpIsDecelerating = false;
            teleOpIsHolding = true;
            teleOpTarget = motionState.pose.withHeading(lockedHeading);
        }
        
        if (!noInput) {
            teleOpIsHolding = false;
            drivetrain.followVector(motionState.makeRobotRelative(new Vector(forward,
                                                                             lateral)),
                                    drivePowerController.computeHeadingCorrectionPower(
                                        lockedHeading, motionState));
        } else if (teleOpIsDecelerating) {
            teleOpIsHolding = false;
            drivetrain.zeroPower();
        } else {
            holdPose(teleOpTarget);
        }
    }
    
    public void robotCentricDrive(double forward, double lateral, double turn) {
        drivetrain.followVector(new Vector(forward, lateral), turn);
    }
}


//    /**
//     * Initializes the robot for tele-op mode,
//     * using the position from the end of the autonomous period.
//     */
//    public void initTeleOp() {
//        localizer.setPose(lastOpModePose);
//        motionTracker.update();
//        motionState = motionTracker.getMotionState();
//        drivetrain.zeroPowerBrakeMode();
//    }
//
//    public void savePoseForTeleOp() {
//        lastOpModePose = new Pose(
//            motionState.position,
//            Math.toDegrees(motionState.heading)
//        );
//    }
//
//    public void setHeadingResetButton(Condition gamepadCondition, double
//    headingDegrees) {
//        this.doWhen(
//            gamepadCondition,
//            () -> localizer.setHeading(headingDegrees)
//        );
//    }
//
//    public void setPoseResetButton(
//        Condition gamepadCondition,
//        double resetX, double resetY, double resetHeading
//    ) {
//        this.doWhen(
//            gamepadCondition,
//            () -> localizer.setPose(resetX, resetY, resetHeading)
//        );
//    }
//
//    /**
//     * Run a basic field-centric tele-op.
//     * <p>
//     * For driver field-centric
//     * <pre><code>
//     * follower.fieldCentricTeleOpDrive(
//     *     -gamepad1.left_stick_y
//     *     -gamepad1.left_stick_x,
//     *     -gamepad1.right_stick_x
//     * );
//     * </code></pre>
//     */
//    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
//        updateMotionState();
//        if (opMode.gamepad1.right_stick_button){ // turning
//            drivetrain.applyBrakingPowers(motionState.makeRobotRelative(new Vector
//            (forward,
//                    lateral)),
//                turn);
//        }
//        else { // simple add
//            drivetrain.followVector(motionState.makeRobotRelative(new Vector(forward,
//            lateral)),
//                turn); // combine into one with condition
//        }
//    }
//
//    public void robotCentricDrive(double y, double x, double turn) {
//        updateMotionState();
//        drivetrain.followVector(new Vector(x, y), turn);
//    }
