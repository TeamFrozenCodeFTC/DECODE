package org.firstinspires.ftc.teamcode.blackice.localizers;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public abstract class Localizer {
    public abstract Pose getPose();
    public abstract Vector getVelocity(); // field-relative velocity
    public abstract double getAngularVelocity();
    public abstract void reset();
    
    public abstract void setCurrentPose(double x, double y, double heading);
    
    public abstract void update(double deltaTime);
    
    /**
     * Converts a robot-relative vector into a field-relative vector.
     * <pre>
     * fieldRelative=R(+θ)×robotRelative
     * </pre>
     * Positive angles are counterclockwise, so this rotates the vector counterclockwise.
     */
    public Vector toFieldRelativeVector(Vector robotRelativeVector) {
        return robotRelativeVector.rotateCounterclockwiseBy(getPose().getHeading());
    }
    
    /**
     * Converts a field-relative vector into a robot-relative vector.
     * <pre>
     * robotRelative=R(−θ)×fieldRelative
     * </pre>
     * Negative angles are clockwise, so this rotates the vector clockwise.
     */
    public Vector toRobotRelativeVector(Vector fieldRelativeVector) {
        return fieldRelativeVector.rotateCounterclockwiseBy(-getPose().getHeading());
    }
}
