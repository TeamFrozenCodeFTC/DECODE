package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Ramp {
    public enum Mode {
        LOAD_TO_SPINDEXER,
        FEED_FROM_SPINDEXER,
        FEED_FROM_INTAKE
    }
    
    public enum MotionState {
        MOVING,
        STATIONARY
    }
    
    private final ServoImplEx servo;
    
    private Mode currentMode = null;
    private MotionState motionState = MotionState.STATIONARY;
    
    private final ElapsedTime moveTimer = new ElapsedTime();
    
    private static final double LOAD_TO_SPINDEXER_POS = 0.361;
    private static final double FEED_FROM_SPINDEXER_POS = 0.725;
    private static final double FEED_FROM_INTAKE_POS = 0.9;
    private static final double MOVE_TIME_SEC = 0.2;
    
    public Ramp(HardwareMap hardwareMap) {
        servo = hardwareMap.get(ServoImplEx.class, "intakeRamp");
    }
    
    public void setMode(Mode newMode) {
        if (newMode == currentMode) return;
        
        currentMode = newMode;
        motionState = MotionState.MOVING;
        moveTimer.reset();
        
        servo.setPwmEnable();
        
        switch (newMode) {
            case LOAD_TO_SPINDEXER:
                servo.setPosition(LOAD_TO_SPINDEXER_POS);
                break;
            case FEED_FROM_SPINDEXER:
                servo.setPosition(FEED_FROM_SPINDEXER_POS);
                break;
            case FEED_FROM_INTAKE:
                servo.setPosition(FEED_FROM_INTAKE_POS);
                break;
        }
    }
    
    public void loadToSpindexer() {
        setMode(Mode.LOAD_TO_SPINDEXER);
    }
    
    public void feedFromSpindexer() {
        setMode(Mode.FEED_FROM_SPINDEXER);
    }
    
    public void feedFromIntake() {
        setMode(Mode.FEED_FROM_INTAKE);
    }
    
    public void update() {
        if (motionState == MotionState.MOVING && moveTimer.seconds() >= MOVE_TIME_SEC) {
            motionState = MotionState.STATIONARY;
        }
    }
    
    public boolean isStationary() {
        return motionState == MotionState.STATIONARY;
    }
    
    public boolean isMoving() {
        return motionState == MotionState.MOVING;
    }
    
    public Mode getMode() {
        return currentMode;
    }
    
    public MotionState getMotionState() {
        return motionState;
    }
    
    public void disable() {
        servo.setPwmDisable();
    }
}
