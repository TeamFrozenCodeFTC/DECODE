package org.firstinspires.ftc.blackice.core.paths.geometry;

import org.firstinspires.ftc.blackice.util.geometry.Vector;

/**
 * A immutable, stateless part of a path, purely positional. It only knows it's tangent heading.
 * The shape of paths.
 */
public interface PathGeometry {
    PathPoint computeClosestPathPointTo(Vector robotPosition, double startingGuess);

    double length();

    PathPoint getEndPathPoint();
    
    PathGeometry reversed();
    PathGeometry mirrored();
}
