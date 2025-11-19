package org.firstinspires.ftc.blackice.core.follower;

import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.core.paths.ImmutablePathBehavior;
import org.firstinspires.ftc.blackice.util.Timeout;

class FollowingTimeouts {
    private final Timeout followingTimeout = new Timeout();
    private final Timeout stuckTimeout = new Timeout();
    private final Timeout pauseTimeout = new Timeout();

    public void start() {
        followingTimeout.resetAndStart();
        stuckTimeout.resetAndStart();
        pauseTimeout.pauseAtZero();
    }
    
    public void done() {
        followingTimeout.pause();
        stuckTimeout.pause();
        pauseTimeout.pauseAtZero();
    }
    
    public void pause() {
        followingTimeout.pause();
        stuckTimeout.pause();
        pauseTimeout.resetAndStart();
    }

    public void resume() {
        followingTimeout.resume();
        stuckTimeout.resume();
        pauseTimeout.pauseAtZero();
    }
    
    public boolean hasTimedOut(
        ImmutablePathBehavior behavior,
        MotionState motionState,
        boolean isFollowing
    ) {
        if (motionState.speed > behavior.stuckVelocityConstraint) {
            stuckTimeout.pauseAtZero();
        }
        else if (isFollowing) {
            stuckTimeout.resume();
        }
        
        return stuckTimeout.hasTimedOut(behavior.stuckTimeoutSeconds)
            || followingTimeout.hasTimedOut(behavior.timeoutSeconds)
            || pauseTimeout.hasTimedOut(behavior.pauseTimeoutSeconds);
    }
}
