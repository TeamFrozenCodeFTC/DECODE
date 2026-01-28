package org.firstinspires.ftc.teamcode.blackice.core.geometry;

import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

/**
 * A point on along a Path that stores info such as point, tangent,
 * and distance.
 */
public class PathPoint {
    public final Vector point;
    public final double tangent;
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

    public PathPoint(Vector point, double tangent, double curvature,
                     double distanceAlongPath,
                     double distanceRemaining,
                     double percentAlongPath, double tValue) {
        this.tangent = tangent;
        this.point = point;
        this.curvature = curvature;
        this.distanceAlongPath = distanceAlongPath;
        this.distanceRemaining = distanceRemaining;
        this.percentAlongPath = percentAlongPath;
        this.tValue = tValue;
    }
}
