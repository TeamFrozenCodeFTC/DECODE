package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

/**
 * Heading interpolation for a path.
 */
@FunctionalInterface
public interface HeadingInterpolator {
    /**
     * (bezierPoint) -> heading
     */
    double interpolate(PathPoint pathPoint);

    /**
     * Offsets the heading interpolator by a given amount.
     */
    default HeadingInterpolator offset(double angle) {
        return pathPoint -> this.interpolate(pathPoint) + angle;
    }
    
    /**
     * Rotates the heading interpolator by 180 degrees.
     */
    default HeadingInterpolator backwards() {
        return offset(Math.PI);
    }
    
    /**
     * Switches the start and end of the heading interpolator. Does nothing if constant
     * or tangent.
     */
    default HeadingInterpolator reversed() {
        return this;
    }
    
    /**
     * Mirrors the heading interpolation along the y-axis.
     */
    default HeadingInterpolator mirrored() {
        return pathPoint -> Math.PI - this.interpolate(pathPoint);
    }
    
    /**
     * The robot will always be facing the direction of the path. This is the default behavior.
     */
    HeadingInterpolator tangent = pathPoint -> pathPoint.tangent;
    
    /**
     * A constant heading along a path.
     */
    static HeadingInterpolator constant(double heading) {
        return pathPoint -> heading;
    }

    /**
     * The robot will transition from the start heading to the end heading.
     */
    static HeadingInterpolator linear(double startHeading, double endHeading) {
        return linear(startHeading, endHeading, 1.00);
    }
    
    /**
     * @param startHeading Heading at the start (degrees)
     * @param endHeading Heading at the end (degrees)
     * @param finishPercent The t parameter (0-1) at which the heading should reach endHeadingDeg
     */
    static HeadingInterpolator linear(double startHeading, double endHeading, double finishPercent) {
        return new HeadingInterpolator() {
            @Override
            public double interpolate(PathPoint pathPoint) {
                double t = Math.min(pathPoint.percentAlongPath / finishPercent, 1.0);
                double deltaHeading = AngleUnit.RADIANS.normalize(endHeading - startHeading);
                return startHeading + deltaHeading * t;
            }
            @Override
            public HeadingInterpolator reversed() {
                return linear(endHeading, startHeading, finishPercent);
            }
        };
    }
    
    /**
     * The robot will always be facing the the given pathPoint.
     */
    static HeadingInterpolator facingPoint(Vector facingPoint) {
        return pathPoint -> {
            Vector direction = facingPoint.minus(pathPoint.point);
            return direction.getAngle();
        };
    }
    
    static HeadingInterpolator tangentToHeading(
        double targetHeadingDegrees,
        double startPercent,
        double endPercent
    ) {
        double targetHeadingRadians = Math.toRadians(targetHeadingDegrees);
        
        return pathPoint -> {
            double t = pathPoint.percentAlongPath;
            
            if (t <= startPercent) {
                // Fully tangent before startPercent
                return pathPoint.tangent;
            } else if (t >= endPercent) {
                // Fully transitioned after endPercent
                return targetHeadingRadians;
            } else {
                // Linearly interpolate between tangent and target heading
                double transitionT = (t - startPercent) / (endPercent - startPercent);
                double tangentHeading = pathPoint.tangent;
                double delta = AngleUnit.RADIANS.normalize(targetHeadingRadians - tangentHeading);
                return tangentHeading + delta * transitionT;
            }
        };
    }
    
}
