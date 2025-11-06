package org.firstinspires.ftc.blackice.core.follower;

import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;

public class VelocityTolerance {
    double positionVelocityTolerance;
    double headingVelocityTolerance;
    
    public VelocityTolerance(double positionVelocityTolerance,
                             double headingVelocityTolerance) {
        this.positionVelocityTolerance = positionVelocityTolerance;
        this.headingVelocityTolerance = headingVelocityTolerance;
    }
    
    public boolean isVelocityWithinTolerance(MotionState motionState) {
        return motionState.speed <= positionVelocityTolerance &&
            Math.abs(
                Math.toDegrees(motionState.angularVelocity)
            ) <= headingVelocityTolerance;
    }
}
