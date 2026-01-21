package org.firstinspires.ftc.teamcode.miniblackice.core.geometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathPoint;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

/**
 * Represents a linear Bezier curve (a straight line) between two points.
 * This is a special case of a Bezier curve with two control points which
 * has enhanced performance for straight lines. Lines can get exact points unlike curves.
 */
public class LineGeometry implements PathGeometry {
    private final double length;
    private final Vector startPoint;
    private final Vector tangent;
    private final PathPoint endPathPoint;

    public LineGeometry(Vector start, Vector end) {
        this.startPoint = start;
        Vector displacement = end.minus(start);
        this.length = displacement.computeMagnitude();
        this.tangent = displacement.dividedBy(length);
        this.endPathPoint = new PathPoint(end, tangent, 0, length, 0, 1, 1);
    }

    public Vector computePointAt(double t) {
        return startPoint.plus(tangent.times(t * length));
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public PathPoint getEndPathPoint() {
        return endPathPoint;
    }

    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double startingGuess) {
        Vector startToPoint = point.minus(startPoint);
        double t = Range.clip(startToPoint.dot(tangent) / length, 0, 1);
        Vector closestPoint = computePointAt(t);
        double distanceAlongPath = t * length;
        return new PathPoint(closestPoint, tangent, 0, distanceAlongPath, length - distanceAlongPath, t, t);
    }

    @Override
    public LineGeometry reversed() {
        return new LineGeometry(endPathPoint.point, startPoint);
    }

    @Override
    public LineGeometry mirrored() {
        return new LineGeometry(startPoint.mirroredAcrossYAxis(), endPathPoint.point.mirroredAcrossYAxis());
    }
}
