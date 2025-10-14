package org.firstinspires.ftc.blackice.core.hardware.localization;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * Represents a snapshot of the robot's motion in terms of its
 * position, heading, delta time, and velocity.
 * Used to store repeatedly used information about the robot's motion.
 * <p>
 * Managed by the {@link MotionTracker}.
 */
public class MotionState {
    public final Pose pose;
    
    public final Vector position;
    public final Vector nextPosition;

    public final double heading; // always in radians
    public final double angularVelocity;
    
    /** Field relative velocity */
    public final Vector velocity;
    public final Vector nextVelocity;
    public final Vector robotRelativeVelocity;
    
    /** Magnitude of current velocity. */
    public final double speed;

    public final Vector previousRobotRelativeVelocity;
    
    /** The time elapsed since the last update. */
    public final double deltaTime;

    public MotionState(
        Pose pose, Vector position, Vector nextPosition,
        double headingRadians,
        double angularVelocity, Vector nextVelocity,
        double deltaTime,
        Vector fieldRelativeVelocity,
        double velocityMagnitude,
        Vector robotRelativeVelocity,
        Vector previousRobotRelativeVelocity
    ) {
        this.pose = pose;
        this.position = position;
        this.nextPosition = nextPosition;
        this.heading = headingRadians;
        this.angularVelocity = angularVelocity;
        this.nextVelocity = nextVelocity;
        this.deltaTime = deltaTime;
        this.velocity = fieldRelativeVelocity;
        this.speed = velocityMagnitude;
        this.robotRelativeVelocity = robotRelativeVelocity;
        this.previousRobotRelativeVelocity = previousRobotRelativeVelocity;
    }
    
//    /**
//     * Predict what the robot's position would be if it slammed on it's brakes right now.
//     */
//    public Vector getPredictedStoppedPosition() {
//        if (predictedStoppedPosition == null) {
//            predictedStoppedPosition = computeStoppedPosition();
//        }
//        return predictedStoppedPosition;
//    }
//
//    private Vector computeStoppedPosition() {
//        Vector robotVelocity = robotRelativeVelocity;
//
//        Vector robotDeltaVelocity = robotVelocity
//            .minus(previousRobotRelativeVelocity);
//
//        Vector predictedRobotVelocity =
//            Kinematics.predictNextLoopVelocity(robotVelocity, robotDeltaVelocity);
//
//        Vector stoppingDisplacement = TuningConstants.BRAKING_DISPLACEMENT
//            .getStoppingDistanceWithVelocity(predictedRobotVelocity);
//
//        return position.plus(toFieldRelativeVector(stoppingDisplacement));
//    }
    
    /**
     * Converts a robot-relative vector into a field-relative vector.
     * <pre>
     * fieldRelative=R(+θ)×robotRelative
     * </pre>
     * Positive angles are counterclockwise, so this rotates the vector counterclockwise.
     */
    public Vector toFieldRelativeVector(Vector robotRelativeVector) {
        return robotRelativeVector.rotateCounterclockwiseBy(heading);
    }
    
    /**
     * Converts a field-relative vector into a robot-relative vector.
     * <pre>
     * robotRelative=R(−θ)×fieldRelative
     * </pre>
     * Negative angles are clockwise, so this rotates the vector clockwise.
     */
    public Vector makeRobotRelative(Vector fieldRelativeVector) {
        return fieldRelativeVector.rotateCounterclockwiseBy(-heading);
    }
}
