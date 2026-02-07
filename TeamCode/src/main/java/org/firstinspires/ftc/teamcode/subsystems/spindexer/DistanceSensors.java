package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensors {
    private static final double HANDOFF_DISTANCE = 3.0;
    private static final double CLEAR_DISTANCE = 6.5;
    
    private final DistancePoller left;
    private final DistancePoller right;
    
    public DistanceSensors(HardwareMap hardwareMap) {
        // NEW
        DistanceSensor leftSensor =
            hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        
        left = new DistancePoller(
            () -> leftSensor.getDistance(DistanceUnit.INCH)
        );
        
        DistanceSensor rightSensor =
            hardwareMap.get(DistanceSensor.class, "leftDistanceSensor");
        
        right = new DistancePoller(
            () -> rightSensor.getDistance(DistanceUnit.INCH)
        );

//        left = new DistancePoller(
//            new Poller<>(
//                () -> hardwareMap
//                    .get(DistanceSensor.class, "leftDistanceSensor")
//                    .getDistance(DistanceUnit.INCH),
//                POLL_MS
//            )
//        );
//
//        right = new DistancePoller(
//            new Poller<>(
//                () -> hardwareMap
//                    .get(DistanceSensor.class, "rightDistanceSensor")
//                    .getDistance(DistanceUnit.INCH),
//                POLL_MS
//            )
//        );
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
    private static final long POLL_MS = 100;
    private static final long COOLDOWN_MS = 5_000; //10_000;
    private static final double FAILING_THRESHOLD = 100;
    
    private final Supplier<Double> readFunction;
    
    double distance = Double.NaN;
    boolean failing;
    
    private long nextPollTime = 0;
    private long nextRecoveryAttempt = 0;
    
    DistancePoller(Supplier<Double> readFunction) {
        this.readFunction = readFunction;
    }
    
    void update() {
        long now = System.currentTimeMillis();
        
        if (failing) {
            if (now < nextRecoveryAttempt) return;
        } else {
            if (now < nextPollTime) return;
        }
        
        double d = readFunction.get();
        
        boolean nowFailing = !Double.isFinite(d) || d >= FAILING_THRESHOLD;
        
        if (nowFailing) {
            failing = true;
            nextRecoveryAttempt = now + (int)(COOLDOWN_MS * (Math.random() * 0.1 + 1));
        } else {
            failing = false;
            distance = d;
            nextPollTime = now + POLL_MS;
        }
    }
}
