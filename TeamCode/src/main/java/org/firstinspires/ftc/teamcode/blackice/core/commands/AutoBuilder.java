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
    private PendingPath lastPath;
    
    public AutoBuilder(Pose startPose, Follower follower) {
        this.currentPose = startPose;
        this.routine = new AutoRoutine(startPose);
        this.follower = follower;
    }
    
    public AutoBuilder addRoutine(AutoRoutine addedRoutine) {
        flushPending();
        routine.addRoutine(addedRoutine);
        Pose endPose = addedRoutine.getEndPose();
        if (endPose != null) {
            currentPose = endPose;
        }
        return this;
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
        lastPath = pendingPath;
        
        currentPose = target;
        return this;
    }
    
    public AutoBuilder lineTo(Pose target, double timeout) {
        return this.lineTo(target).withTimeout(timeout);
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
        lastPath = pendingPath;
        
        currentPose = endPoint;
        return this;
    }
    
    public AutoBuilder holdLastPath() {
        pendingPath = lastPath;
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
    
    public AutoBuilder withTimeout(double timeout) {
        if (pendingPath != null) {
            pendingPath.timeout = timeout;
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
    
//    public AutoBuilder holdLastPath() {
//        if (pendingPath == null) return this;
//
//        routine.add(
//            new FollowPathCommand(
//                pendingPath.geometry,
//                pendingPath.headingInterpolator,
//                pendingPath.finishCondition,
//                follower
//            ).withTimeout(pendingPath.timeout)
//        );
//
//        return this;
//    }
    
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
            ).withTimeout(pendingPath.timeout)
        );
        
        pendingPath = null;
    }
    
    private static class PendingPath {
        final PathGeometry geometry;
        Pose endPose;
        HeadingInterpolator headingInterpolator;
        PathFinishCondition finishCondition =
            PathFinishConditions.withinBrakingDistance();
        double timeout = 6;
        
        PendingPath(PathGeometry geometry, Pose endPose) {
            this.geometry = geometry;
            this.endPose = endPose;
            this.headingInterpolator = HeadingInterpolator.constant(endPose.getHeading());
        }
    }
}
