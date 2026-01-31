package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {
    // Paddle positions
    private static final double OPEN_LEFT = 0.952;
    private static final double OPEN_RIGHT = 0.172;
    private static final double CLOSED_LEFT = 0.600;
    private static final double CLOSED_RIGHT = 0.515;
    
    // Ramp positions
    private static final double LOAD_TO_SPINDEXER_POS = 0.369;
    private static final double FEED_FROM_SPINDEXER_POS = 0.725;
    private static final double FEED_FROM_INTAKE_POS = 0.9;
    
    private static final double PADDLE_MOVE_TIME_SEC = 0.40;
    private static final double RAMP_MOVE_TIME_SEC = 0.40;
    
    private final ServoImplEx leftPaddle;
    private final ServoImplEx rightPaddle;
    private final ServoImplEx rampServo;
    
    private PaddleMode paddleMode = null;
    private RampMode rampMode = null;
    
    private MotionState paddleMotion = MotionState.STATIONARY;
    private MotionState rampMotion = MotionState.STATIONARY;
    
    private final ElapsedTime paddleTimer = new ElapsedTime();
    private final ElapsedTime rampTimer = new ElapsedTime();
    
    private PaddleIntent paddleIntent = PaddleIntent.NONE;
    private RampIntent rampIntent = RampIntent.NONE;
    
    public enum MotionState {
        MOVING,
        STATIONARY
    }
    
    public enum PaddleMode {
        OPEN,
        CLOSED
    }
    
    public enum RampMode {
        LOAD_TO_SPINDEXER,
        FEED_FROM_SPINDEXER,
        FEED_FROM_INTAKE
    }
    
    private enum PaddleIntent {
        NONE,
        OPEN,
        CLOSE
    }
    
    private enum RampIntent {
        NONE,
        LOAD_TO_SPINDEXER,
        FEED_FROM_SPINDEXER,
        FEED_FROM_INTAKE
    }
    
    public Transfer(HardwareMap hardwareMap) {
        leftPaddle = hardwareMap.get(ServoImplEx.class, "leftPaddle");
        rightPaddle = hardwareMap.get(ServoImplEx.class, "rightPaddle");
        rampServo = hardwareMap.get(ServoImplEx.class, "intakeRamp");
    }
    
    public void openPaddles() {
        paddleIntent = PaddleIntent.OPEN;
    }
    
    public void closePaddles() {
        paddleIntent = PaddleIntent.CLOSE;
    }
    
    public void loadToSpindexer() {
        rampIntent = RampIntent.LOAD_TO_SPINDEXER;
    }
    
    public void feedFromSpindexer() {
        rampIntent = RampIntent.FEED_FROM_SPINDEXER;
    }
    
    public void feedFromIntake() {
        rampIntent = RampIntent.FEED_FROM_INTAKE;
    }
    
    public void update() {
        updateMotionStates();
        
        if (paddleIntent != PaddleIntent.NONE && rampMotion == MotionState.STATIONARY) {
            switch (paddleIntent) {
                case OPEN:
                    setPaddles(PaddleMode.OPEN);
                    break;
                case CLOSE:
                    setPaddles(PaddleMode.CLOSED);
                    break;
            }
            paddleIntent = PaddleIntent.NONE;
        }
        
        if (rampIntent != RampIntent.NONE && paddleMotion == MotionState.STATIONARY) {
            switch (rampIntent) {
                case LOAD_TO_SPINDEXER:
                    setRamp(RampMode.LOAD_TO_SPINDEXER);
                    break;
                case FEED_FROM_SPINDEXER:
                    setRamp(RampMode.FEED_FROM_SPINDEXER);
                    break;
                case FEED_FROM_INTAKE:
                    setRamp(RampMode.FEED_FROM_INTAKE);
                    break;
            }
            rampIntent = RampIntent.NONE;
        }
    }
    
    private void updateMotionStates() {
        if (paddleMotion == MotionState.MOVING &&
            paddleTimer.seconds() >= PADDLE_MOVE_TIME_SEC) {
            paddleMotion = MotionState.STATIONARY;
        }
        
        if (rampMotion == MotionState.MOVING &&
            rampTimer.seconds() >= RAMP_MOVE_TIME_SEC) {
            rampMotion = MotionState.STATIONARY;
        }
    }
    
    private void setPaddles(PaddleMode mode) {
        if (mode == paddleMode) return;
        
        paddleMode = mode;
        paddleMotion = MotionState.MOVING;
        paddleTimer.reset();
        
        switch (mode) {
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
    
    private void setRamp(RampMode mode) {
        if (mode == rampMode) return;
        
        rampMode = mode;
        rampMotion = MotionState.MOVING;
        rampTimer.reset();
        
        rampServo.setPwmEnable();
        
        switch (mode) {
            case LOAD_TO_SPINDEXER:
                rampServo.setPosition(LOAD_TO_SPINDEXER_POS);
                break;
            case FEED_FROM_SPINDEXER:
                rampServo.setPosition(FEED_FROM_SPINDEXER_POS);
                break;
            case FEED_FROM_INTAKE:
                rampServo.setPosition(FEED_FROM_INTAKE_POS);
                break;
        }
    }
    
    public boolean arePaddlesMoving() {
        return paddleMotion == MotionState.MOVING;
    }
    
    public boolean isRampMoving() {
        return rampMotion == MotionState.MOVING;
    }
    
    public PaddleMode getPaddleMode() {
        return paddleMode;
    }
    
    public RampMode getRampMode() {
        return rampMode;
    }
}
