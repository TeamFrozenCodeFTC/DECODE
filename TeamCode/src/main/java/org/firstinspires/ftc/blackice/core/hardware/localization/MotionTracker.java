package org.firstinspires.ftc.blackice.core.hardware.localization;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

import java.util.List;

// Turn MotionTracker into localizer?
/**
 * Updates the robot's {@link MotionState} based on the {@link Localizer}'s data.
 */
public class MotionTracker {
    public final Localizer localizer;
    
    private final ElapsedTime stuckDetectedTimer = new ElapsedTime(0);
    
    public MotionTracker(HardwareMap hardwareMap, Localizer localizer) {
        this.localizer = localizer;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    private double lastTime = System.nanoTime();
    private Vector previousRobotRelativeVelocity = new Vector(0,0);
    private Vector previousVelocity = new Vector(0,0);
    private MotionState motionState;
    
    public void setCurrentPose(Pose newPose) {
        localizer.setPose(newPose);
        update();
    }
    
    public void setCurrentHeading(double heading) {
        localizer.setHeading(heading);
        update();
    }
    
    public void update() {
        localizer.update();

        Vector position = new Vector(
            localizer.getX(),
            localizer.getY()
        );
        Vector fieldRelativeVelocity = new Vector(
            localizer.getFieldVelocityX(),
            localizer.getFieldVelocityY()
        );
        Vector robotRelativeVelocity = fieldRelativeVelocity.rotateClockwiseBy(localizer.getHeading());
        double deltaTime = calculateDeltaTime();
        motionState = new MotionState(
            new Pose(position, localizer.getHeading()),
            position,
            position.plus(fieldRelativeVelocity.times(deltaTime)),
            localizer.getHeading(),
            localizer.getAngularVelocity(),
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
    
    private double calculateDeltaTime() {
        double currentTime = System.nanoTime();
        double deltaTime = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;
        return deltaTime;
    }
    
    public void resetStuckTimer() {
        stuckDetectedTimer.reset();
    }
}
