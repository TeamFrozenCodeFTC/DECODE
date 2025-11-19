package org.firstinspires.ftc.blackice.core.follower;

import org.firstinspires.ftc.blackice.util.geometry.Pose;

public class PoseTolerance {
    double positionTolerance;
    double headingTolerance;
    
    public PoseTolerance(double positionTolerance, double headingTolerance) {
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
    }
    
    public boolean isPoseWithinTolerance(Pose target, Pose current) {
        double positionError = target.getPosition().distanceTo(current.getPosition());
        double headingError = Math.abs(
            Math.toDegrees(
                Math.atan2(
                    Math.sin(Math.toRadians(target.getHeading() - current.getHeading())),
                    Math.cos(Math.toRadians(target.getHeading() - current.getHeading()))
                )
            )
        );
        return positionError <= positionTolerance && headingError <= headingTolerance;
    }
}
