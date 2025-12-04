package org.firstinspires.ftc.blackice.simple;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

public abstract class Localizer2 {
    private double lastTime = System.nanoTime();
    private double deltaTime = 0.0;
    
    abstract Pose getPose();
    abstract Pose getFieldVelocity();
    
    // deltaTime

    abstract void updateInternals(double deltaTime);
    abstract void reset();
    
    abstract void setInternalPose(double x, double y, double heading);
    
    public void setCurrentPose(Pose pose) {
        setInternalPose(pose.getPosition().getX(), pose.getPosition().getY(), pose.getHeading());
        updateInternals();
    }
    
    public double getDeltaTime() {
        return deltaTime;
    }

    public void updateInternals() {
        deltaTime = calculateDeltaTime();
        updateInternals(deltaTime);
    }
    
    private double calculateDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        return deltaTime;
    }
    
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
    public Vector makeRobotRelative(Vector fieldRelativeVector) {
        return fieldRelativeVector.rotateCounterclockwiseBy(-getPose().getHeading());
    }
}
