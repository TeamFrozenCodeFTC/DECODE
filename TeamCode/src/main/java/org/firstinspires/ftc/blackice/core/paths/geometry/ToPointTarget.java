package org.firstinspires.ftc.blackice.core.paths.geometry;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * A target point from the reference of a previous starting point.
 * Is like LineSegment but just goes towards endPoint instead of forcing it to be
 * on the line.
 */
public class ToPointTarget extends LinearGeometry {
    public ToPointTarget(Vector previousTarget, Vector targetPoint) {
        super(previousTarget, targetPoint);
    }
    
    @Override
    public PathPoint computeClosestPathPointTo(Vector point, double previousTValue) {
        double distanceRemaining = Range.clip(endPathPoint.point.minus(point).dotProduct(tangent), 0, length);
        double distanceAlongPath = length - distanceRemaining;
        double percentAlong = distanceAlongPath / length;
        
        return new PathPoint(endPathPoint.point, tangent, 0, distanceAlongPath, distanceRemaining, percentAlong, percentAlong);
    }
    
    @Override
    public PathGeometry reversed() {
        return new ToPointTarget(endPathPoint.point, startPoint);
    }
    
    @Override
    public ToPointTarget mirrored() {
        return new ToPointTarget(startPoint.mirroredAcrossYAxis(), endPathPoint.point.mirroredAcrossYAxis());
    }
}
