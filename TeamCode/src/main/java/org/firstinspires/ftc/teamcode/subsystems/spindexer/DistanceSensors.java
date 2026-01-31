package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors {
    private static final double HANDOFF_DISTANCE = 3.0;
    private static final double CLEAR_DISTANCE = 6.5;
    
    private static final long POLL_MS = 100;
   
    private final DistancePoller left;
    private final DistancePoller right;
    
    public DistanceSensors(HardwareMap hardwareMap) {
        left = new DistancePoller(
            new Poller<>(
                () -> hardwareMap
                    .get(DistanceSensor.class, "leftDistanceSensor")
                    .getDistance(DistanceUnit.INCH),
                POLL_MS
            )
        );
        
        right = new DistancePoller(
            new Poller<>(
                () -> hardwareMap
                    .get(DistanceSensor.class, "rightDistanceSensor")
                    .getDistance(DistanceUnit.INCH),
                POLL_MS
            )
        );
    }
    
    public void update() {
        left.update();
        right.update();
    }
    
    public double getLeftDistance() {
        return left.distance;
    }
    
    public double getRightDistance() {
        return right.distance;
    }
    
    public boolean isLeftFailing() {
        return left.failing;
    }
    
    public boolean isRightFailing() {
        return right.failing;
    }
    
    public boolean isArtifactInHandoffZone() {
        return left.distance < HANDOFF_DISTANCE
            || right.distance < HANDOFF_DISTANCE;
    }
    
    public boolean isSpindexerClear() {
        return left.distance > CLEAR_DISTANCE
            && right.distance > CLEAR_DISTANCE;
    }
}

class DistancePoller {
    private static final long COOLDOWN_MS = 10000;
    private static final double FAILING_THRESHOLD = 100;
    
    final Poller<Double> sensor;
    double distance;
    boolean failing;
    
    DistancePoller(Poller<Double> sensor) {
        this.sensor = sensor;
    }
    
    void update() {
        distance = sensor.poll();
        
        boolean nowFailing = distance >= FAILING_THRESHOLD;
        
        if (!failing && nowFailing) {
            failing = true;
            sensor.delayNextPoll(COOLDOWN_MS);
        } else if (failing && !nowFailing) {
            failing = false;
        }
    }
}
