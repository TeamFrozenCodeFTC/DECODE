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
    public final Drivetrain drivetrain;
    public final Pose startingPose;
    private final double pauseWhenVoltageBelow;
    public boolean teleOpIsHolding = false;
    public boolean teleOpIsDecelerating = true;
    public Pose teleOpTarget;
    private PathBehavior defaultPathBehavior;
    private boolean lowVoltage = false;

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
    
    public void addDefaultPathBehavior(PathBehavior behavior) {
        defaultPathBehavior = defaultPathBehavior.mergeWith(behavior);
    }
    
    public void holdPose(Pose pose) {
        drivePowerController.holdPose(pose, getMotionState());
    }
    
    public boolean isWithinBraking(Pose pose) {
        return drivePowerController.computeHoldPower(pose.getPosition(),
                                                   getMotionState()).computeMagnitude() < 1;
    }

    public boolean isAt(Pose pose, PoseTolerance tolerance) {
        return tolerance.isPoseWithinTolerance(pose, getCurrentPose());
    }
    
    public boolean isStoppedAt(Pose pose, PoseTolerance tolerance,
                               VelocityTolerance velocityTolerance) {
        return isAt(pose, tolerance) && velocityTolerance.isVelocityWithinTolerance(getMotionState());
    }
    
    public boolean isStoppedAt(Pose pose) {
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
    
    public double getVoltage() {
        return drivePowerController.getVoltage();
    }
    
    Double lockedHeading = null;
    
    public void lockHeadingAt(Double heading) {
         lockedHeading = heading;
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
//    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
//        MotionState motionState = getMotionState();
//
//        boolean noInput = forward == 0 && lateral == 0 && turn == 0;
//
//        if (noInput && !teleOpIsHolding && !teleOpIsDecelerating) {
//            teleOpIsDecelerating = true;
//        }
//
//        if (motionState.speed < 0.1 && teleOpIsDecelerating && !teleOpIsHolding) {
//            teleOpIsDecelerating = false;
//            teleOpIsHolding = true;
//            teleOpTarget =
//                motionState.pose.withHeading(Math.toDegrees(motionState.heading));
//        }
//
//        if (!noInput || teleOpIsDecelerating) {
//            teleOpIsHolding = false;
//            if (lockedHeading != null) {
//                turn = drivePowerController.computeHeadingCorrectionPower(
//                    lockedHeading, motionState);
//            }
//            drivetrain.followVector(
//                motionState.makeRobotRelative(new Vector(forward, lateral)), turn);
////        } else if (teleOpIsDecelerating) {
////            teleOpIsHolding = false;
////            drivetrain.zeroPower();
//        } else {
//            if (lockedHeading != null) {
//                teleOpTarget = teleOpTarget.withHeading(lockedHeading);
//            }
//            holdPose(teleOpTarget);
//        }
//    }
    
    public void fieldCentricTeleOpDrive(double forward, double lateral, double turn) {
        MotionState motion = getMotionState();
        boolean noInput = forward == 0 && lateral == 0 && turn == 0;
        
        if (noInput) {
            if (!teleOpIsHolding && !teleOpIsDecelerating) {
                teleOpIsDecelerating = true;
            }
            
            if (motion.speed < 0.1 && teleOpIsDecelerating && !teleOpIsHolding) {
                teleOpIsDecelerating = false;
                teleOpIsHolding = true;
                teleOpTarget = motion.pose.withHeading(Math.toDegrees(motion.heading));
            }
        } else {
            teleOpIsHolding = false;
            teleOpIsDecelerating = false;
        }
        
        if (!noInput || teleOpIsDecelerating) {
            if (lockedHeading != null) {
                turn = drivePowerController.computeHeadingCorrectionPower(lockedHeading, motion);
            }
            drivetrain.followVector(motion.makeRobotRelative(new Vector(forward, lateral)), turn);
        } else {
            if (lockedHeading != null) {
                teleOpTarget = teleOpTarget.withHeading(Math.toDegrees(lockedHeading));
            }
            holdPose(teleOpTarget);
        }
    }
    
    
    public void robotCentricDrive(double forward, double lateral, double turn) {
        drivetrain.followVector(new Vector(forward, lateral), turn);
    }
}
