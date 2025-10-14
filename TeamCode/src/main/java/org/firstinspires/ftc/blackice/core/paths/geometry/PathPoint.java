package org.firstinspires.ftc.blackice.core.paths.geometry;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * A point on along a Path that stores info such as point, tangent,
 * and distance.
 */
public class PathPoint {
    public final Vector point;
    public final Vector tangent;
    public final Vector perpendicularVector;
    public final double curvature;
    public final double distanceAlongPath;
    public final double distanceRemaining;
    public final double percentAlongPath;
    
    /**
     * The parametric t value between 0 to 1. Used only for internal
     * calculations. 0.5 tValue is not perfectly half way along the
     * path. 0.5 {@link #percentAlongPath} is equal to half way along
     * the path. Recommended to use {@link #percentAlongPath} for
     * triggering things along the path.
     */
    public final double tValue;
    
    public PathPoint(Vector point, Vector tangent, double curvature,
                     double distanceAlongPath,
                     double distanceRemaining,
                     double percentAlongPath, double tValue) {
        this.tangent = tangent;
        this.perpendicularVector = tangent.perpendicularLeft();
        this.point = point;
        this.curvature = curvature;
        this.distanceAlongPath = distanceAlongPath;
        this.distanceRemaining = distanceRemaining;
        this.percentAlongPath = percentAlongPath;
        this.tValue = tValue;
    }
}
