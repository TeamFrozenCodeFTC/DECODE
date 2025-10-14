package org.firstinspires.ftc.blackice.core.paths.profile;

import static org.firstinspires.ftc.blackice.util.Utils.getOrDefault;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.Logger;
import org.firstinspires.ftc.blackice.util.Validator;

/**
 * A velocity profile that has attributes that resemble a trapezoid. Can accelerate, cruise,
 * decelerate and optional continue cruising after decelerating.
 */
public class TrapezoidalVelocityProfile implements VelocityProfile {
    private final double maxVelocity;
    private final @Nullable Double twoTimesAcceleration;
    private final @Nullable Double twoTimesDeceleration;
    private final double endingVelocityCruiseDistance;
    private final double endingVelocitySquared;
    private final double endingVelocity;

    /**
     * Accelerates to maxVelocity and then decelerates until it reaches the given ending velocity
     * where it cruises there for endingVelocityCruiseDistance inches. Linearly increases the target
     * velocity while accelerating or decelerating.
     * <p>
     * All parameters allow null.
     *
     * @param acceleration                 (inches/s^2) How fast the robot accelerates. Must be
     *                                     positive. If null, it will not accelerate.
     * @param maxVelocity                  (inches/s) The maximum velocity the robot can reach. Must
     *                                     be positive. If null, it will not have a maximum
     *                                     velocity.
     * @param deceleration                 (inches/s^2) How fast the robot decelerates. Must be
     *                                     positive. If null, it will not decelerate.
     * @param endingVelocity               (inches/s) The velocity the robot decelerates to. Must be
     *                                     positive. If null, it will end with zero velocity.
     * @param endingVelocityCruiseDistance (inches) The distance the robot travels at the ending
     *                                     velocity. Must be zero, positive, or null. If null, it
     *                                     will be zero.
     */
    public TrapezoidalVelocityProfile(@Nullable Acceleration acceleration,
                                      @Nullable Double maxVelocity,
                                      @Nullable Acceleration deceleration,
                                      @Nullable Double endingVelocity, // finalVelocity
                                      @Nullable Double endingVelocityCruiseDistance) {
        this.maxVelocity =
            Validator.ensureGreaterThanZero(getOrDefault(maxVelocity, Double.POSITIVE_INFINITY));
        this.endingVelocity = Validator.ensureGreaterThanZero(getOrDefault(endingVelocity,
            0));
        this.endingVelocitySquared = this.endingVelocity * this.endingVelocity;
        
        if (deceleration != null) {
            this.twoTimesDeceleration =
                2 * Validator.ensurePositiveSign(deceleration.compute(this.maxVelocity,
                    this.endingVelocity));
        } else {
            this.twoTimesDeceleration = null;
        }
        
        if (acceleration != null) {
            this.twoTimesAcceleration =
                2 * Validator.ensurePositiveSign(acceleration.compute(0, this.maxVelocity));
        } else {
            this.twoTimesAcceleration = null;
        }
        
        this.endingVelocityCruiseDistance =
            Validator.ensureGreaterThanZero(getOrDefault(endingVelocityCruiseDistance, 0));
        
        if (this.endingVelocity > this.maxVelocity) {
            Logger.warnWithStack("Ending velocity exceeds maximum velocity.");
        }
    }
    
    @Override
    public double computeTargetVelocity(double distanceAlongPath,
                                        double distanceRemaining) {
        double velocityToFitAcceleration;
        if (twoTimesAcceleration == null) {
            velocityToFitAcceleration = Double.POSITIVE_INFINITY;
        }
        else {
            velocityToFitAcceleration = Math.sqrt(twoTimesAcceleration * distanceAlongPath);
        }
        
        double maxVelocityToFitAcceleration = Math.min(maxVelocity, velocityToFitAcceleration);
        
        if (twoTimesDeceleration == null) {
            return maxVelocityToFitAcceleration;
        }
        
        double distanceToDecelerate =
            distanceRemaining - endingVelocityCruiseDistance;

        if (distanceToDecelerate <= 0) {
            return endingVelocity;
        }
        
        double rawVelocity =
            Math.sqrt(Math.max(0,
                               endingVelocitySquared + twoTimesDeceleration * distanceToDecelerate));
        return Math.min(maxVelocityToFitAcceleration, rawVelocity);
    }
    
    
    /**
     * A type of acceleration by accelerating or decelerating between velocities either by time
     * (s), distance (inch) or a constant (inches/s^2).
     */
    @FunctionalInterface
    public interface Acceleration {
        double compute(double startingVelocity, double endingVelocity);
        
        static Acceleration constant(double acceleration) {
            return (startingVelocity, endingVelocity) -> acceleration;
        }

        /**
         * Decelerations far away from coast velocity result in jitter of reversing powers
         * or early decelerations. If you want smooth deceleration, use coast
         * deceleration. If you want fast deceleration set deceleration to none and
         * let the positional PID handle it.
         */
        @Deprecated
        static Acceleration toReachInTime(double seconds) {
            return (startingVelocity, endingVelocity) ->
                (endingVelocity - startingVelocity) / (seconds);
        }
        
        /**
         * Decelerations far away from coast velocity result in jitter of reversing powers
         * or early decelerations. If you want smooth deceleration, use coast
         * deceleration. If you want fast deceleration set deceleration to none and
         * let the positional PID handle it.
         */
        @Deprecated
        static Acceleration toReachOverDistance(double distance) {
            return (startingVelocity, endingVelocity) ->
                (endingVelocity * endingVelocity - startingVelocity * startingVelocity) / (2 * distance);
        }
    }
}
