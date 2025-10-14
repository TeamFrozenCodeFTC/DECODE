package org.firstinspires.ftc.blackice.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timeout {
    private final ElapsedTime baseTimer = new ElapsedTime();
    private double accumulatedTime = 0.0;
    private boolean isPaused = false;

    public void reset() {
        accumulatedTime = 0.0;
        baseTimer.reset();
        isPaused = false;
    }

    public void pause() {
        if (!isPaused) {
            accumulatedTime += baseTimer.seconds();
            isPaused = true;
        }
    }
    
    public void pauseAtZero() {
        reset();
        pause();
    }

    public void resume() {
        if (isPaused) {
            baseTimer.reset();
            isPaused = false;
        }
    }

    public double seconds() {
        return accumulatedTime + (isPaused ? 0.0 : baseTimer.seconds());
    }
    
    public boolean hasTimedOut(double timeoutSeconds) {
        return timeoutSeconds != -1 && this.seconds() > timeoutSeconds;
    }

    public boolean isPaused() {
        return isPaused;
    }
}
