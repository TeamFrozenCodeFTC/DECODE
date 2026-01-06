package org.firstinspires.ftc.teamcode.blackice.core;


import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;

public class PoseTolerance {
    double positionTolerance;
    double headingTolerance;
    
    public PoseTolerance(double positionTolerance, double headingTolerance) {
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
    }
    
    public boolean atPose(Pose target, Pose current) {
        double positionError = target.getPosition().distanceTo(current.getPosition());
        double headingError = Math.abs(
            Math.toDegrees(
                Math.atan2(
                    Math.sin(target.getHeading() - current.getHeading()),
                    Math.cos(target.getHeading() - current.getHeading())
                )
            )
        );
        return positionError <= positionTolerance && headingError <= headingTolerance;
    }
}
