//package org.firstinspires.ftc.teamcode.blackice.core.commands;
//
//import org.firstinspires.ftc.teamcode.blackice.core.FollowPathCommand;
//import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
//
//public class AutoBuilder {
//    private final AutoRoutine routine;
//
//    private Pose currentPose;
//    private PendingPath pendingPath;
//
//    public AutoBuilder(Pose startPose) {
//        this.currentPose = startPose;
//        this.routine = new AutoRoutine(startPose);
//    }
//
//    public AutoRoutine build() {
//        flushPending();
//        return routine;
//    }
//
//    public AutoBuilder lineTo(Pose target) {
//        flushPending();
//        pendingPath = new PendingPath(currentPose, target);
//        currentPose = target;
//        return this;
//    }
//
//    public AutoBuilder until(PathFinishCondition condition) {
//        if (pendingPath != null) {
//            pendingPath.finishCondition = condition;
//        }
//        return this;
//    }
//
//    public AutoBuilder stop() {
//        if (pendingPath != null) {
//            pendingPath.finishCondition = PathFinishConditions.stoppedAtEnd();
//        }
//        return this;
//    }
//
//    private void flushPending() {
//        if (pendingPath == null) return;
//
//        routine.add(
//            new FollowPathCommand(
//                new LineGeometry(
//                    pendingPath.start.getPosition(),
//                    pendingPath.end.getPosition()
//                ),
//                HeadingInterpolator.constant(
//                    pendingPath.end.getHeading()
//                ),
//                pendingPath.finishCondition
//            )
//        );
//
//        pendingPath = null;
//    }
//
//    private static class PendingPath {
//        Pose start;
//        Pose end;
//        PathFinishCondition finishCondition =
//            PathFinishConditions.withinBrakingDistance();
//
//        PendingPath(Pose start, Pose end) {
//            this.start = start;
//            this.end = end;
//        }
//    }
//}
