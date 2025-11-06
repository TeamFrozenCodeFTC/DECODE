package org.firstinspires.ftc.blackice.core.hardware.localization;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

public abstract class Localizer2 {
    abstract double getX();
    abstract double getY();
    abstract double getHeading();
    
    abstract double getFieldVelocityX();
    abstract double getFieldVelocityY();
    
    abstract double getAngularVelocity();
    
    abstract void updateInternals();
    abstract void reset();
    
    abstract void setInternalPose(double x, double y, double heading);
    public void setCurrentPose(Pose pose) {
        setInternalPose(pose.getPosition().getX(), pose.getPosition().getY(), pose.getHeading());
        updateInternals();
    }
    
    private final ElapsedTime stuckDetectedTimer = new ElapsedTime(0);

    private Vector previousRobotRelativeVelocity = new Vector(0, 0);
    private Vector previousVelocity = new Vector(0,0);
    private MotionState motionState;
    
    public void updateInternals(double deltaTime) {
        updateInternals();
        
        Vector position = new Vector(
            getX(),
            getY()
        );
        Vector fieldRelativeVelocity = new Vector(
            getFieldVelocityX(),
            getFieldVelocityY()
        );
        Vector robotRelativeVelocity = fieldRelativeVelocity.rotateClockwiseBy(getHeading());
        motionState = new MotionState(
            new Pose(position, getHeading()),
            position,
            position.plus(fieldRelativeVelocity.times(deltaTime)),
            getHeading(),
            getAngularVelocity(),
            fieldRelativeVelocity.times(2).minus(previousVelocity),
            deltaTime,
            fieldRelativeVelocity,
            robotRelativeVelocity.computeMagnitude(),
            robotRelativeVelocity,
            previousRobotRelativeVelocity
        );
        previousVelocity = fieldRelativeVelocity;
        previousRobotRelativeVelocity = robotRelativeVelocity;
    }
    
    public MotionState getMotionState() {
        return motionState;
    }
}
