package org.firstinspires.ftc.teamcode.subsystems.spindexer;


import java.util.function.Supplier;

public class Poller<T> {
    private final Supplier<T> readFunction;
    private final long pollIntervalMs;
    
    private long nextPollTime = 0;
    private T lastValue;
    
    public Poller(Supplier<T> readFunction, long pollIntervalMs) {
        this.readFunction = readFunction;
        this.pollIntervalMs = pollIntervalMs;
    }
    
    public T poll() {
        long now = System.currentTimeMillis();
        
        if (now >= nextPollTime) {
            lastValue = readFunction.get();
            nextPollTime = now + pollIntervalMs;
        }
        
        return lastValue;
    }
    
    public void delayNextPoll(long delayMs) {
        nextPollTime += delayMs;
    }
}
