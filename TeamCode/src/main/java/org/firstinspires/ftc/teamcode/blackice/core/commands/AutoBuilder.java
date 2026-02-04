package org.firstinspires.ftc.teamcode.blackice.core.commands;

import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
import org.firstinspires.ftc.teamcode.blackice.core.Follower;
import org.firstinspires.ftc.teamcode.blackice.core.HeadingInterpolator;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.BezierGeometry;
import org.firstinspires.ftc.teamcode.blackice.core.geometry.PathGeometry;
import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.miniblackice.core.geometry.LineGeometry;

import java.util.function.BooleanSupplier;

public class AutoBuilder {
    private final AutoRoutine routine;
    private final Follower follower;
    
    private Pose currentPose;
    private PendingPath pendingPath;
    
    public AutoBuilder(Pose startPose, Follower follower) {
        this.currentPose = startPose;
        this.routine = new AutoRoutine(startPose);
        this.follower = follower;
    }
    
    public AutoRoutine build() {
        flushPending();
        routine.add(Command.singleAction(follower::stop));
        return routine;
    }
    
    public AutoBuilder lineTo(Pose target) {
        flushPending();
        
        pendingPath = new PendingPath(
            new LineGeometry(
                currentPose.getPosition(),
                target.getPosition()
            ),
            target
        );
        
        currentPose = target;
        return this;
    }
    
    public AutoBuilder curveTo(Pose controlPoint, Pose endPoint) {
        flushPending();
        
        pendingPath = new PendingPath(
            new BezierGeometry(
                currentPose.getPosition(),
                controlPoint.getPosition(),
                endPoint.getPosition()
            ),
            endPoint
        );
        
        currentPose = endPoint;
        return this;
    }
    
    public AutoBuilder until(PathFinishCondition condition) {
        if (pendingPath != null) {
            pendingPath.finishCondition = condition;
        }
        return this;
    }
    
    public AutoBuilder until(BooleanSupplier condition) {
        if (pendingPath != null) {
            pendingPath.finishCondition = (f,p) -> condition.getAsBoolean();
        }
        return this;
    }
    
    public AutoBuilder stop() {
        if (pendingPath != null) {
            pendingPath.finishCondition =
                PathFinishConditions.stoppedAtEnd();
        }
        return this;
    }
    
    public AutoBuilder holdLastPath() {
        if (pendingPath == null) return this;
        
        routine.add(
            new FollowPathCommand(
                pendingPath.geometry,
                pendingPath.headingInterpolator,
                pendingPath.finishCondition,
                follower
            )
        );
        
        return this;
    }
    
    public AutoBuilder addAction(Runnable action) {
        flushPending();
        routine.add(Command.singleAction(action));
        return this;
    }
    
    public AutoBuilder linearHeadingInterpolation() {
        if (pendingPath != null) {
            pendingPath.headingInterpolator =
                HeadingInterpolator.linear(currentPose.getHeading(),
                                           pendingPath.endPose.getHeading());
        }
        return this;
    }
    
    private void flushPending() {
        if (pendingPath == null) return;
        
        routine.add(
            new FollowPathCommand(
                pendingPath.geometry,
                pendingPath.headingInterpolator,
                pendingPath.finishCondition,
                follower
            )
        );
        
        pendingPath = null;
    }
    
    private static class PendingPath {
        final PathGeometry geometry;
        Pose endPose;
        HeadingInterpolator headingInterpolator;
        PathFinishCondition finishCondition =
            PathFinishConditions.withinBrakingDistance();
        
        PendingPath(PathGeometry geometry, Pose endPose) {
            this.geometry = geometry;
            this.endPose = endPose;
            this.headingInterpolator = HeadingInterpolator.constant(endPose.getHeading());
        }
    }
}
