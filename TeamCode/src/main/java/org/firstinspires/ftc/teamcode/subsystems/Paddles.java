package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Paddles {
    private static final double OPEN_LEFT = 0.842;
    private static final double OPEN_RIGHT = 0.172;
    private static final double CLOSED_LEFT = 0.479;
    private static final double CLOSED_RIGHT = 0.515;
    private static final double MOVE_TIME_SEC = 0.20;
    
    public enum Mode {
        OPEN,
        CLOSED
    }
    
    public enum MotionState {
        MOVING,
        STATIONARY
    }
    
    private final ServoImplEx leftPaddle;
    private final ServoImplEx rightPaddle;
    
    private Mode mode = null;
    private MotionState motionState = MotionState.STATIONARY;
    
    private final ElapsedTime moveTimer = new ElapsedTime();
    
    public Paddles(HardwareMap hardwareMap) {
        leftPaddle = hardwareMap.get(ServoImplEx.class, "leftPaddle");
        rightPaddle = hardwareMap.get(ServoImplEx.class, "rightPaddle");
    }
    
    public void setPosition(Mode newMode) {
        if (newMode == mode) return;
        
        mode = newMode;
        motionState = MotionState.MOVING;
        moveTimer.reset();
        
        switch (newMode) {
            case OPEN:
                leftPaddle.setPosition(OPEN_LEFT);
                rightPaddle.setPosition(OPEN_RIGHT);
                break;
            
            case CLOSED:
                leftPaddle.setPosition(CLOSED_LEFT);
                rightPaddle.setPosition(CLOSED_RIGHT);
                break;
        }
    }
    
    public void open() {
        setPosition(Mode.OPEN);
    }
    
    public void close() {
        setPosition(Mode.CLOSED);
    }
    
    public void disablePower() {
        leftPaddle.setPwmDisable();
        rightPaddle.setPwmDisable();
    }
    
    public void update() {
        if (motionState == MotionState.MOVING &&
            moveTimer.seconds() >= MOVE_TIME_SEC) {
            motionState = MotionState.STATIONARY;
        }
    }
    
    public boolean isMoving() {
        return motionState == MotionState.MOVING;
    }
    
    public boolean isStationary() {
        return motionState == MotionState.STATIONARY;
    }
    
    public Mode getMode() {
        return mode;
    }
}
