package org.firstinspires.ftc.blackice.core.follower;

import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.core.paths.Path;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathPoint;

class PathState {
    public final Path path;
    public final PathPoint closestPathPoint;
    public final MotionState motionState;
    public final double tangentialVelocity;
    
    public PathState(Path path, PathPoint closestPoint, MotionState motionState) {
        this.path = path;
        this.closestPathPoint = closestPoint;
        this.motionState = motionState;
        this.tangentialVelocity =
            motionState.nextVelocity.dotProduct(closestPathPoint.tangent);
    }
}
