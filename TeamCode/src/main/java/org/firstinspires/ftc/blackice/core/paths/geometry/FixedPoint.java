package org.firstinspires.ftc.blackice.core.paths.geometry;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * A target point that does not move. Has no length or distance. Does NOT work to
 * interpolate heading, accelerate, or decelerate.
 */
public class FixedPoint implements PathGeometry {
    private final PathPoint endPoint;
    
    public FixedPoint(Vector point) {
        endPoint = new PathPoint(point, Vector.ZERO, 0, 0, 0, 1, 1);
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector robotPosition,
                                               double startingGuess) {
        return new PathPoint(endPoint.point, Vector.ZERO, 0, 0,
                             endPoint.point.minus(robotPosition).computeMagnitude(),
                             1, 1);
    }
    
    @Override
    public double length() {
        return 0;
    }
    
    @Override
    public PathPoint getEndPathPoint() {
        return endPoint;
    }
    
    @Override
    public FixedPoint reversed() {
        return this;
    }
    
    @Override
    public PathGeometry mirrored() {
        return new FixedPoint(endPoint.point.mirroredAcrossYAxis());
    }
}
