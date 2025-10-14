package org.firstinspires.ftc.blackice.core.paths.routines;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.geometry.Point;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.blackice.core.paths.HeadingInterpolator;
import org.firstinspires.ftc.blackice.core.paths.Path;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.geometry.BezierCurve;
import org.firstinspires.ftc.blackice.core.paths.geometry.FixedPoint;
import org.firstinspires.ftc.blackice.core.paths.geometry.LineSegment;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathGeometry;
import org.firstinspires.ftc.blackice.core.paths.geometry.ToPointTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 * Keeps track of the previous target heading and position to create one path after another in the
 * Follower.
 */
public class PathRoutineBuilder {
    @FunctionalInterface
    interface RoutineStepFactory {
        RoutineStep toRoutineStep(Pose previousPose);
    }
    
    private final Supplier<Pose> currentPoseSupplier;
    
    public PathRoutineBuilder(Supplier<Pose> currentPoseSupplier) {
        this.currentPoseSupplier = currentPoseSupplier;
    }
    
    private PathBehavior defaultPathBehavior = new PathBehavior();
    
    private final List<RoutineStepFactory> steps = new ArrayList<>();
    public Pose startingPose;
    private Consumer<PathFactory> queuedChanges = pathFactory -> {};
    
    private @Nullable PathFactory currentPath = null;
    private boolean currentPathEditable = true;

    public PathRoutineBuilder withStartPose(Pose startingPose) {
        this.startingPose = startingPose;
        return this;
    }
    
    public PathRoutineBuilder withDefaultBehavior(PathBehavior defaultPathBehavior) {
        this.defaultPathBehavior = this.defaultPathBehavior.mergeWith(defaultPathBehavior);
        return this;
    }
    
    private PathRoutineBuilder addPath(PathFactory path) {
        queuedChanges.accept(path);
        queuedChanges = pathFactory -> {};
        steps.add(path);
        currentPath = path;
        currentPathEditable = true;
        return this;
    }
    
    public PathRoutine build(Pose startPose) {
        Pose previousEndPose = startPose;
        
        ArrayList<RoutineStep> steps = new ArrayList<>();
        for (RoutineStepFactory step : this.steps) {
            RoutineStep routineStep = step.toRoutineStep(previousEndPose);
            steps.add(routineStep);
            if (routineStep instanceof Path) {
                previousEndPose = ((Path) routineStep).endPose;
            }
        }
        
        return new PathRoutine(steps);
    }
    
    public PathRoutine build() {
        if (startingPose == null) {
            throw new IllegalArgumentException("No starting pose for this PathSequence");
        }
        return build(startingPose);
    }
    
    public boolean hasStartingPose() {
        return startingPose != null;
    }
    
    public PathRoutineBuilder addRoutine(PathRoutineBuilder other) {
        for (RoutineStepFactory step : other.copy().steps) {
            if (!(step instanceof PathFactory)) {
                this.steps.add(step);
                currentPathEditable = false;
                continue;
            }
            addPath((PathFactory) step);
        }
        return this;
    }
    
    public PathRoutineBuilder copy() {
        PathRoutineBuilder builder = new PathRoutineBuilder(currentPoseSupplier);
        for (RoutineStepFactory step : this.steps) {
            if (!(step instanceof PathFactory)) {
                this.steps.add(step);
                builder.currentPathEditable = false;
                continue;
            }
            PathFactory path = (PathFactory) step;
            PathFactory copy = new PathFactory(
                path.geometrySupplier,
                path.headingInterpolatorSupplier,
                path.behavior.clone(),
                path.targetHeading
            );
            builder.addPath(copy);
        }
        return builder;
    }
    
    public PathRoutineBuilder copyReverse() {
        PathRoutineBuilder builder = new PathRoutineBuilder(currentPoseSupplier);
        for (int i = this.steps.size() - 1; i >= 0; i--) {
            RoutineStepFactory step = this.steps.get(i);
            if (!(step instanceof PathFactory)) {
                this.steps.add(step);
                builder.currentPathEditable = false;
                continue;
            }
            PathFactory path = (PathFactory) step;
            PathFactory copy = new PathFactory(
                previousPoint -> path.geometrySupplier.get(previousPoint).reversed(),
                (previousHeading,
                targetHeading) -> path.headingInterpolatorSupplier.get(previousHeading,targetHeading).reversed(),
                path.behavior.clone(),
                path.targetHeading
            );
            builder.addPath(copy);
        }
        return builder;
    }
    
    private PathRoutineBuilder addPath(PathGeometry geometry) {
        return addPath(start -> geometry);
    }
    
    private PathRoutineBuilder addPath(PathFactory.GeometrySupplier geometry) {
        return addPath(geometry, null);
    }
    
    private PathRoutineBuilder addPath(PathFactory.GeometrySupplier geometry,
                                       @Nullable Double targetHeading) {
        return addPath(new PathFactory(
            geometry,
            PathFactory.HeadingInterpolatorSupplier.CONSTANT,
            defaultPathBehavior,
            targetHeading
        ));
    }
    
    /**
     * Runs the given action between the previous step and the next step.
     */
    public PathRoutineBuilder runAction(Action action) {
        steps.add(startPose -> action);
        currentPathEditable = false;
        return this;
    }
    
    private PathFactory getEditablePath() {
        if (currentPath == null) {
            throw new IllegalStateException("Cannot use this method before adding a path.");
        }
        if (!currentPathEditable) {
            throw new IllegalStateException("The current path is not editable. " +
                                                "Consider moving this method " +
                                                "somewhere else");
        }
        return currentPath;
    }
    
    /**
     * Sets the behavior of the last added path.
     */
    public PathRoutineBuilder withBehavior(PathBehavior behavior) {
        getEditablePath().overrideBehavior(behavior);
        return this;
    }
    
    /**
     * Goes from the previous target pose to the new given pose.
     */
    public PathRoutineBuilder toPose(Pose pose) {
        return addPath(
            previousPoint -> new ToPointTarget(previousPoint, pose.getPosition()),
            pose.getNullableHeading()
        );
    }
    public PathRoutineBuilder toPose(double x, double y, double heading) {
        return toPose(new Pose(x, y, heading));
    }
    public PathRoutineBuilder toPose(double x, double y) {
        return toPose(new Pose(x, y));
    }
    
    public PathRoutineBuilder toPoint(Point point) {
        return toPose(new Pose(point, null));
    }

    /**
     * Makes the last added path start from the given pose.
     * <p>
     * Useful if you don't want a path to start from the previous path's endPose.
     */
    public PathRoutineBuilder from(Pose pose) {
        if (currentPath == null) {
            withStartPose(pose);
        }
        else {
            getEditablePath().setStartingPose(pose);
        }
        return this;
    }
    
    public PathRoutineBuilder from(PathFactory.PoseProvider poseProvider) {
        PathFactory path = getEditablePath();
        path.isRuntimePath = true;
        path.startingPoseSupplier = poseProvider;
        return this;
    }
    
    /**
     * Edits ether the last added path or next path to be added (only if there is no
     * last path).
     */
    private PathRoutineBuilder editPath(Consumer<PathFactory> edit) {
        if (currentPath != null && !currentPathEditable) {
            edit.accept(currentPath);
        }
        else {
            queuedChanges = queuedChanges.andThen(edit);
        }
        return this;
    }
    
    /**
     * Sets the last added path so that it starts from a future pose (such as the robot's
     * current position) instead of the previous path's end pose. If no last path, then
     * this will edit the next added path.
     * <p>
     * Note: makes this path a runtime path, meaning it will not be built until
     * follower.startFollowing is called.
     */
    public PathRoutineBuilder fromFuturePose(PathFactory.PoseProvider poseProvider) {
        return editPath(path -> {
            path.isRuntimePath = true;
            path.startingPoseSupplier = poseProvider;
        });
    }

    /**
     * Follows a line between the previous target pose to the new given pose.
     * @see #toPose(Pose)
     */
    public PathRoutineBuilder lineTo(Pose pose) {
        return addPath(
            previousPoint -> new LineSegment(previousPoint, pose.getPosition()),
            pose.getNullableHeading()
        );
    }
    public PathRoutineBuilder lineTo(double x, double y, double heading) {
        return lineTo(new Pose(x, y, heading));
    }
    public PathRoutineBuilder lineTo(double x, double y) {
        return lineTo(new Pose(x, y));
    }
    
    /**
     * Set the function that tells the robot what heading it should be, at a given point.
     */
    public PathRoutineBuilder withHeadingInterpolator(HeadingInterpolator headingInterpolator) {
        getEditablePath().headingInterpolatorSupplier = (previous, target) -> headingInterpolator;
        return this;
    }
    
    /**
     * Set the headingInterpolator based off the previous heading from the last path
     * and the current target heading.
     */
    public PathRoutineBuilder withHeadingInterpolatorSupplier(PathFactory.HeadingInterpolatorSupplier headingInterpolator) {
        getEditablePath().headingInterpolatorSupplier = headingInterpolator;
        return this;
    }
    
    // setLinearHeadingInterpolation
    public PathRoutineBuilder withHeadingInterpolationTo(double heading) {
        getEditablePath().targetHeading = heading;
        return withHeadingInterpolatorSupplier(PathFactory.HeadingInterpolatorSupplier.LINEAR);
    }
    
    // setConstantHeading
    public PathRoutineBuilder withConstantHeading(double constantHeading) {
        getEditablePath().targetHeading = constantHeading;
        return withHeadingInterpolatorSupplier(PathFactory.HeadingInterpolatorSupplier.CONSTANT);
    }
    
    public PathRoutineBuilder withTangentHeading() {
        return withHeadingInterpolator(HeadingInterpolator.tangent);
    }
    
    public PathRoutineBuilder withHeadingInterpolation() {
        return withHeadingInterpolationTo(requireTargetHeading());
    }
    
    public PathRoutineBuilder withConstantHeading() {
        return withConstantHeading(requireTargetHeading());
    }
    
//    HeadingInterpolator routineHeadingInterpolator;
//
//    // TODO setHeadingInterpolator for whole path
//    public PathRoutineBuilder setRoutineHeadingInterpolator(HeadingInterpolator routineHeadingInterpolator) {
//        this.routineHeadingInterpolator = routineHeadingInterpolator;
//    }
    
    // TODO turn tangent and then back to a heading
    // TODO turn at end of path (by distanceRemaining left on path)
    
    private double requireTargetHeading() {
        Double heading = getCurrentPath().targetHeading;
        if (heading == null) {
            throw new IllegalStateException("No heading available from path endpoint.");
        }
        return heading;
    }
    
    /**
     * Stops at the end of the last added path.
     */
    public PathRoutineBuilder stop() {
        getCurrentPath().behavior.brakeToStop();
        return this;
    }

    public PathRoutineBuilder turnTo(double heading) {
        addPath(FixedPoint::new, Math.toRadians(heading))
            .stop();
        currentPathEditable = false;
        return this;
    }

    /**
     * Creates a path that follows a line from the robot's previous target point to the given x.
     */
    public PathRoutineBuilder lineToX(double x) {
        return addPath(previousPoint -> new LineSegment(previousPoint, previousPoint.withX(x)));
    }
    
    /**
     * Creates a path that follows a line from the robot's previous target point to the given y.
     */
    public PathRoutineBuilder lineToY(double y) {
        return addPath(previousPoint -> new LineSegment(previousPoint, previousPoint.withY(y)));
    }
  
    // TODO
//    public PathSequence setCurrentPose(Pose currentPose) {
//        this.previousEndPose = currentPose;
//        drafts.add(new DeferredPath())
//        return this;
//    }
    
    private PathFactory getCurrentPath() {
        if (currentPath == null) {
            throw new IllegalStateException("There is no current path to use this method on.");
        }
        return currentPath;
    }

    public PathRoutineBuilder curveTo(Vector midpoint, Vector endPoint) {
        // P1 = 2M - (p0+p2)/2
        // We want to construct a Bézier curve that starts at
        return addPath(new PathFactory(
            previousPoint -> new BezierCurve(previousPoint,
                                             midpoint.times(2).minus(previousPoint.plus(endPoint).times(0.5)),
                                             endPoint),
            PathFactory.HeadingInterpolatorSupplier.TANGENT,
            new PathBehavior(),
            null
        ));
    }


//    public SegmentSequenceBuilder bezierCurve(double[][] controlPoints, PathBehaviorConfig behavior) {
//
//        segments.add(new BezierCurve(controlPoints));
//        paths = paths.add(new Path(new BezierCurve(controlPoints)))
//        previousTargetPoint = curve.getEndPoint();
//        return this;
//    }

//    /**
//     * Creates a Bezier curve path segment that is approximated by a series of line segments.
//     * This is theoretically a lot faster because it is precalculated and will never skip part of
//     * the path because it is sequential.
//     *
//     * @param inchPerPoint the distance between each point in inches. Usually 1-2 inches.
//     */
//    public SegmentSequenceBuilder linedBezierCurve(double[][] controlPoints,
//                                                   double inchPerPoint) {
//        BezierCurve curve = new BezierCurve(controlPoints);
//
//        int numPoints = (int) Math.ceil(curve.length() / inchPerPoint);
//        if (numPoints < 2) numPoints = 2;
//        Vector[] points = new Vector[numPoints + 1];
//        for (int i = 0; i <= numPoints; i++) {
//            double t = (double) i / numPoints;
//            points[i] = curve.calculatePointAt(t);
//        }
//        for (int i = 0; i < points.length - 1; i++) {
//            Line segment = new Line(points[i], points[i + 1]);
//            segments.add(segment);
//        }
//        previousTargetPoint = points[points.length - 1];
//
//        return this;
//    }
}
