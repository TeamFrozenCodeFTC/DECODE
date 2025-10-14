package org.firstinspires.ftc.blackice.core.paths;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathGeometry;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathPoint;
import org.firstinspires.ftc.blackice.core.paths.routines.RoutineStep;

/**
 * An immutable path with geometry of where the robot should go and behavior of how the Follower
 * should follow the path.
 */
public class Path implements RoutineStep {
    public final PathGeometry geometry;
    public final HeadingInterpolator headingInterpolator;
    public final ImmutablePathBehavior behavior;
    
    public final Pose endPose;
    
    public Path(PathGeometry geometry, HeadingInterpolator headingInterpolator,
                ImmutablePathBehavior behavior) {
        this.geometry = geometry;
        this.headingInterpolator = headingInterpolator;
        this.behavior = behavior;
        PathPoint endPathPoint = geometry.getEndPathPoint();
        this.endPose = new Pose(
            endPathPoint.point,
            headingInterpolator.interpolate(endPathPoint)
        );
    }

    public Path reversed() {
        return new Path(
            geometry.reversed(),
            headingInterpolator.reversed(),
            behavior
        );
    }

    public Path mirrored() {
        return new Path(
            geometry.mirrored(),
            headingInterpolator.mirrored(),
            behavior
        );
    }
}
