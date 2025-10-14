package org.firstinspires.ftc.blackice.core.paths;

import org.firstinspires.ftc.blackice.core.paths.routines.RoutineStep;

import java.util.function.Supplier;

public class RuntimePath implements RoutineStep {
    private final Supplier<Path> factory;
    
    public RuntimePath(Supplier<Path> factory) {
        this.factory = factory;
    }
    
    @Override
    public Path resolve() {
        return factory.get();
    }
}
