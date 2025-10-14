package org.firstinspires.ftc.blackice.core.paths.geometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * Represents a linear Bezier curve (a straight line) between two points.
 * This is a special case of a Bezier curve with two control points which
 * has enhanced performance for straight lines. Lines can get exact points unlike curves.
 */
public class LineSegment extends LinearGeometry {
    /**
     * Constructs a line between two points.
     * <p>
     * Points must be at least 1e-6 away from each other.
     */
    public LineSegment(Vector start, Vector end) {
        super(start, end);
        if (length < 1e-6) {
            throw new IllegalArgumentException("Line too short");
        }
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double startingGuess) {
        Vector startToPoint = point.minus(startPoint);
        double t = Range.clip(startToPoint.dotProduct(tangent) / length, 0, 1);
        Vector closestPoint = computePointAt(t);
        double distanceAlongPath = t * length;
        return new PathPoint(closestPoint, tangent, 0, distanceAlongPath, length - distanceAlongPath, t, t);
    }
    
    @Override
    public LineSegment reversed() {
        return new LineSegment(endPathPoint.point, startPoint);
    }
    
    @Override
    public LineSegment mirrored() {
        return new LineSegment(startPoint.mirroredAcrossYAxis(), endPathPoint.point.mirroredAcrossYAxis());
    }
}
