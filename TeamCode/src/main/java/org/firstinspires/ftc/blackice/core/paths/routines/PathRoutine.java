package org.firstinspires.ftc.blackice.core.paths.routines;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.actions.Action;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.core.paths.Path;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * An ordered list of {@link Path}s and {@link Action}s (or {@link RoutineStep}s) that
 * make up a part of an autonomous routine.
 */
public class PathRoutine {
    private final List<RoutineStep> steps;
    public final @Nullable Pose endPose;
    public final int amountOfSteps;
    
    PathRoutine(List<RoutineStep> steps) {
        this.steps = Collections.unmodifiableList(steps);
        
        Pose endPose = null;
        for (int i = this.steps.size() - 1; i >= 0; i--) {
            RoutineStep step = this.steps.get(i);
            if (step instanceof Path) {
                endPose = ((Path) step).endPose;
                break;
            }
        }
        this.endPose = endPose;
        this.amountOfSteps = this.steps.size();
    }

    public PathRoutine reversed() {
        List<RoutineStep> reversedSteps = new ArrayList<>(this.steps);
        for (int i = 0; i < steps.size(); i++) {
            RoutineStep step = steps.get(i);
            if (step instanceof Path) {
                steps.set(i, ((Path) step).reversed());
            }
        }
        Collections.reverse(reversedSteps);
        return new PathRoutine(reversedSteps);
    }
    
    public PathRoutine mirrored() {
        List<RoutineStep> steps = new ArrayList<>(this.steps);
        for (int i = 0; i < steps.size(); i++) {
            RoutineStep step = steps.get(i);
            if (step instanceof Path) {
                steps.set(i, ((Path) step).mirrored());
            }
        }
        return new PathRoutine(steps);
    }
    
    public static PathRoutine empty() {
        return new PathRoutine(Collections.emptyList());
    }
    
    public List<RoutineStep> getSteps() {
        return steps;
    }
    
    public RoutineStep getStep(int index) {
        return steps.get(index);
    }
    
    public PathRoutine extended(List<RoutineStep> moreSteps) {
        List<RoutineStep> combined = new ArrayList<>(this.steps);
        combined.addAll(moreSteps);
        return new PathRoutine(combined);
    }
}
