package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors {
    private static final double HANDOFF_DISTANCE = 3.0;
    private static final double CLEAR_DISTANCE = 6.5;
    private static final long POLL_MS = 0;
    private static final long COOLDOWN_MS = 3000;
    
    private static final double FAILING_THRESHOLD = 100; // 322.5197 static, 2580.1
    // unplugged

    private double leftDistance;
    private double rightDistance;
    
    private boolean leftDistanceFail = false;
    private boolean rightDistanceFail = false;
    
    private final Poller<Double> leftSensor;
    private final Poller<Double> rightSensor;
    
    public DistanceSensors(HardwareMap hardwareMap) {
        DistanceSensor leftDistanceSensor =
            hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        DistanceSensor rightDistanceSensor =
            hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        leftSensor =
            new Poller<>(() -> leftDistanceSensor.getDistance(DistanceUnit.INCH)
                , POLL_MS, 10);
        rightSensor =
            new Poller<>(() -> rightDistanceSensor.getDistance(DistanceUnit.INCH)
                , POLL_MS, 30);
    }
    
    public void update() {
        rightDistance = rightSensor.poll();
        if (!rightDistanceFail) {
            if (rightDistance >= FAILING_THRESHOLD) {
                rightDistanceFail = true;
                rightSensor.delayNextPoll(COOLDOWN_MS);
            }
        }
        else {
            if (rightDistance < FAILING_THRESHOLD) {
                rightDistanceFail = false;
            }
        }
        
        leftDistance = leftSensor.poll();
        if (!leftDistanceFail) {
            if (leftDistance >= FAILING_THRESHOLD) {
                leftDistanceFail = true;
                leftSensor.delayNextPoll(COOLDOWN_MS);
            }
        }
        else {
            if (leftDistance < FAILING_THRESHOLD) {
                leftDistanceFail = false;
            }
        }
    }
    
    public boolean isArtifactInHandoffZone() {
        return leftDistance < HANDOFF_DISTANCE || rightDistance < HANDOFF_DISTANCE;
    }
    
    public boolean isSpindexerClear() {
        return leftDistance > CLEAR_DISTANCE && rightDistance > CLEAR_DISTANCE;
    }
    
    public boolean leftDistanceSensorIsFailing() {
        return leftDistanceFail;
    }
    
    public boolean rightDistanceSensorIsFailing() {
        return rightDistanceFail;
    }
    
    public double getLeftDistance() {
        return leftDistance;
    }
    
    public double getRightDistance() {
        return rightDistance;
    }
}
