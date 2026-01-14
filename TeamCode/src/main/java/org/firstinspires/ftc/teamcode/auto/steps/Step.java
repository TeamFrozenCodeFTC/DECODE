package org.firstinspires.ftc.teamcode.auto.steps;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class Step {
    private static final double DEFAULT_TIMEOUT = 5.0; // seconds
    
    Runnable onEnter;
    Runnable onLoop;
    BooleanSupplier isDone;
    Runnable done = () -> {};
    
    double timeoutSeconds = DEFAULT_TIMEOUT;
    ElapsedTime timer = new ElapsedTime();
    boolean entered = false;
    
    public Step(Runnable onLoop, BooleanSupplier isDone) {
        this(null, onLoop, isDone);
    }
    
    public Step(Runnable onEnter, Runnable onLoop, BooleanSupplier isDone,
                Runnable done) {
        this.onEnter = onEnter;
        this.onLoop = onLoop;
        this.isDone = isDone;
        this.done = done;
    }
    
    public Step(Runnable onEnter, Runnable onLoop, BooleanSupplier isDone) {
        this.onEnter = onEnter;
        this.onLoop = onLoop;
        this.isDone = isDone;
    }
    
    public Step withTimeout(double seconds) {
        this.timeoutSeconds = seconds;
        return this;
    }
    
    public void run() {
        if (!entered) {
            entered = true;
            timer.reset();
            if (onEnter != null) onEnter.run();
        }
        
        onLoop.run();
    }
    
    public boolean isFinished() {
        return isDone.getAsBoolean() || timer.seconds() >= timeoutSeconds;
    }
    
    public static Step timeout(double seconds) {
        return new Step(
            () -> {},
            () -> false
        ).withTimeout(seconds);
    }
}
