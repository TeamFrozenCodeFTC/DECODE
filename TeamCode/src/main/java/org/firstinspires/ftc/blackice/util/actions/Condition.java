package org.firstinspires.ftc.blackice.util.actions;

@FunctionalInterface
public interface Condition {
    boolean isTrue();

    Condition ALWAYS = () -> true;
    Condition NEVER = () -> false;

    default Condition and(Condition other) {
        if (other == NEVER || this == NEVER) {
            return NEVER;
        }
        if (this == ALWAYS) {
            return other;
        }
        if (other == ALWAYS) {
            return this;
        }
        
        return () -> this.isTrue() && other.isTrue();
    }

    default Condition or(Condition other) {
        if (this == ALWAYS || other == ALWAYS) {
            return ALWAYS;
        }
        if (this == NEVER) {
            return other;
        }
        if (other == NEVER) {
            return this;
        }
        
        return () -> this.isTrue() || other.isTrue();
    }
}
