package org.firstinspires.ftc.blackice.core.paths.routines;

public interface RoutineStep {
    default RoutineStep resolve() { return this; }
}
