package org.firstinspires.ftc.blackice.core.paths;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.actions.ActionLoop;
import org.firstinspires.ftc.blackice.core.paths.profile.VelocityProfile;

import java.lang.reflect.Field;

public class ImmutablePathBehavior {
    public final ActionLoop actionLoop;
    
    public final PathBehavior.StopMode stop;
    
    public final double stoppedVelocityConstraint;
    public final double stoppedAngularVelocityConstraint;
    
    public final double stuckVelocityConstraint;
    public final double stuckTimeoutSeconds;
    public final boolean cancelPathIfStuck;
    
    public final double pauseTimeoutSeconds;
    public final double timeoutSeconds;

    public final @Nullable VelocityProfile velocityProfile;
    
    public final double maxTurnPower;
    
    public final boolean allowReversePowerDeceleration;
    
    public ImmutablePathBehavior(ActionLoop actionLoop, PathBehavior.StopMode stopAtEnd,
                                 double stoppedVelocityConstraint,
                                 double stoppedAngularVelocityConstraint,
                                 double stuckVelocityConstraint, double stuckTimeoutSeconds,
                                 boolean cancelPathIfStuck, double pauseTimeoutSeconds, double timeoutSeconds,
                                 @Nullable VelocityProfile velocityProfile,
                                 double maxTurnPower,
                                 boolean allowReversePowerDeceleration) {
        this.actionLoop = actionLoop;
        this.stop = stopAtEnd;
        this.stoppedVelocityConstraint = stoppedVelocityConstraint;
        this.stoppedAngularVelocityConstraint = stoppedAngularVelocityConstraint;
        this.stuckVelocityConstraint = stuckVelocityConstraint;
        this.stuckTimeoutSeconds = stuckTimeoutSeconds;
        this.cancelPathIfStuck = cancelPathIfStuck;
        this.pauseTimeoutSeconds = pauseTimeoutSeconds;
        this.timeoutSeconds = timeoutSeconds;
        this.velocityProfile = velocityProfile;
        this.maxTurnPower = maxTurnPower;
        this.allowReversePowerDeceleration = allowReversePowerDeceleration;
    }
    
    @Deprecated // replaces null fields with default values
    public PathBehavior toBuilder() {
        PathBehavior builder = new PathBehavior();
        try {
            for (Field field : ImmutablePathBehavior.class.getDeclaredFields()) {
                field.setAccessible(true);
                Object value = field.get(this);
                field.set(builder, value);
            }
        } catch (IllegalAccessException e) {
            throw new RuntimeException("Reflection error while copying behavior", e);
        }
        return builder;
    }
}
