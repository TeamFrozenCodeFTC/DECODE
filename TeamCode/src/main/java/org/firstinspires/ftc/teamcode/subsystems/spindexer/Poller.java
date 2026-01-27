package org.firstinspires.ftc.teamcode.subsystems.spindexer;


import java.util.function.Supplier;

public class Poller<T> {
    private final long pollIntervalMs;
    private final long offsetMs;
    private long lastPollTime = 0;
    private T lastValue;
    private final Supplier<T> readFunction;
    
    public Poller(Supplier<T> readFunction, long pollIntervalMs,
                  long offsetMs) {
        this.readFunction = readFunction;
        this.pollIntervalMs = pollIntervalMs;
        this.offsetMs = offsetMs;
    }
    
    public T poll() {
        long now = System.currentTimeMillis();
        if (now - lastPollTime + offsetMs >= pollIntervalMs) {
            lastValue = readFunction.get();
            lastPollTime = now;
        }
        return lastValue;
    }
    
    public void delayNextPoll(long ms) {
        lastPollTime += ms;
    }
    
    public void reset() {
        lastPollTime = 0;
        lastValue = null;
    }
}
