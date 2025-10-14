package org.firstinspires.ftc.blackice.util.actions;

import org.firstinspires.ftc.blackice.core.paths.routines.RoutineStep;

@FunctionalInterface
public interface Action extends RoutineStep {
    void execute();
    
    /**
     * Executes the action if the condition is true.
     */
    default void executeWhen(boolean condition) {
        if (condition) {
            execute();
        }
    }
    
    /**
     * Combines this action with another action, executing both in sequence.
     */
    default Action andThen(Action other) {
        if (other == EMPTY) {
            return this;
        }
        if (this == EMPTY) {
            return other;
        }
        
        return () -> {
            this.execute();
            other.execute();
        };
    }
    
    Action EMPTY = () -> {};
}
