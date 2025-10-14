package org.firstinspires.ftc.blackice.core.paths.routines;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;
import org.firstinspires.ftc.blackice.core.paths.HeadingInterpolator;
import org.firstinspires.ftc.blackice.core.paths.ImmutablePathBehavior;
import org.firstinspires.ftc.blackice.core.paths.Path;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.core.paths.geometry.PathGeometry;
import org.firstinspires.ftc.blackice.core.paths.RuntimePath;
import org.firstinspires.ftc.blackice.util.Utils;

/**
 * Internal class for creating a path in a routine.
 * <p>
 * Used by {@link PathRoutineBuilder}
 */
class PathFactory implements PathRoutineBuilder.RoutineStepFactory {
    @FunctionalInterface
    interface GeometrySupplier {
        PathGeometry get(Vector previousPoint);
    }
    
    @FunctionalInterface
    public interface PoseProvider {
        Pose get(Pose previousPose);
    }
    
    @FunctionalInterface
    public interface HeadingInterpolatorSupplier {
        HeadingInterpolator get(double previousHeading, double targetHeading);
        
        HeadingInterpolatorSupplier CONSTANT =
            (previous, target) -> HeadingInterpolator.constant(target);
        HeadingInterpolatorSupplier TANGENT =
            (previous, target) -> HeadingInterpolator.tangent;
        HeadingInterpolatorSupplier LINEAR = HeadingInterpolator::linear;
    }
    
    GeometrySupplier geometrySupplier;
    HeadingInterpolatorSupplier headingInterpolatorSupplier;
    PathBehavior behavior;
    
    @Nullable Double targetHeading;
    
    boolean isRuntimePath = false;
    PoseProvider startingPoseSupplier = previousPose -> previousPose;

    PathFactory(GeometrySupplier geometrySupplier,
                HeadingInterpolatorSupplier headingInterpolatorSupplier,
                PathBehavior behavior,
                @Nullable Double targetHeading) {
        this.geometrySupplier = geometrySupplier;
        this.headingInterpolatorSupplier = headingInterpolatorSupplier;
        this.behavior = behavior.clone();
        this.targetHeading = targetHeading;
    }
    
    void overrideBehavior(PathBehavior behavior) {
        this.behavior = behavior.mergeWith(behavior);
    }
    
    void setStartingPose(Pose startingPose) {
        startingPoseSupplier = previousPose -> startingPose;
    }
    
    private RuntimePath toRuntimePath(Pose previousPose) {
        ImmutablePathBehavior finalBehavior = behavior.build();
        return new RuntimePath(() -> {
            Pose startingPose = startingPoseSupplier.get(previousPose).completeWith(previousPose);
            return new Path(
                geometrySupplier.get(startingPose.getPosition()),
                headingInterpolatorSupplier.get(
                    startingPose.getHeading(),
                    Utils.getOrDefault(targetHeading, startingPose.getHeading())
                ),
                finalBehavior
            );
        });
    }
    
    @Override
    public RoutineStep toRoutineStep(Pose previousPose) {
        if (isRuntimePath) {
            return this.toRuntimePath(previousPose);
        }
        
        Pose startingPose = startingPoseSupplier.get(previousPose);
       
        Vector startingPathPosition = (startingPose == null)
            ? previousPose.getPosition()
            : startingPose.getPosition();
        
        double startingPathHeading =
            (startingPose == null || !startingPose.hasHeading())
            ? previousPose.getHeading()
            : startingPose.getHeading();
            
        HeadingInterpolator headingInterpolator = (targetHeading != null)
            ? headingInterpolatorSupplier.get(startingPathHeading,
                                              targetHeading)
            : headingInterpolatorSupplier.get(startingPathHeading,
                                            startingPathHeading);
        
        return new Path(
            geometrySupplier.get(startingPathPosition),
            headingInterpolator,
            behavior.build()
        );
    }
}
