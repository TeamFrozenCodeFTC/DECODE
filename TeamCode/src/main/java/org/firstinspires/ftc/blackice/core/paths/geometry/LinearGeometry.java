package org.firstinspires.ftc.blackice.core.paths.geometry;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * Abstract class representing a linear geometric path.
 * Provides common functionality for paths that are linear in nature.
 */
abstract class LinearGeometry implements PathGeometry {
    protected final double length;
    protected final Vector startPoint;
    protected final Vector tangent;
    protected final PathPoint endPathPoint;
    
    protected LinearGeometry(Vector start, Vector end) {
        this.startPoint = start;
        Vector displacement = end.minus(start);
        this.length = displacement.computeMagnitude();
        this.tangent = displacement.divide(length);
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
}
