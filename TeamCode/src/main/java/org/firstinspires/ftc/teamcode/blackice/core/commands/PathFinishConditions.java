package org.firstinspires.ftc.teamcode.blackice.core.commands;

import org.firstinspires.ftc.teamcode.blackice.core.Follower;

import java.util.function.BooleanSupplier;

public final class PathFinishConditions {
    public static PathFinishCondition withinBrakingDistance() {
        return Follower::isWithinBraking;
    }
    
    public static PathFinishCondition stoppedAtEnd() {
        return Follower::isStoppedAt;
    }
    
    public static PathFinishCondition custom(
        BooleanSupplier condition
    ) {
        return (follower, geometry) -> condition.getAsBoolean();
    }
    
    private PathFinishConditions() {}
}
