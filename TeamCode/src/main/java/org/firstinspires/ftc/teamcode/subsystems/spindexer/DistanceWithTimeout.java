package org.firstinspires.ftc.teamcode.subsystems.spindexer;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.atomic.AtomicBoolean;

public class DistanceWithTimeout implements Callable<Double> {
    ExecutorService executor;
    AtomicBoolean working;
    double lastValue;
    int timeoutMillis;
    DistanceSensor sensor;
    DistanceUnit distanceUnit;

    public DistanceWithTimeout(DistanceSensor sensor, DistanceUnit distanceUnit, int timeoutMillis) {
        this.sensor = sensor;
        this.timeoutMillis = timeoutMillis;
        this.lastValue = 0;
        this.working = new AtomicBoolean(false);
        this.executor = Executors.newSingleThreadExecutor();
        this.distanceUnit = distanceUnit;
    }

    public double getDistance() {
        double result = lastValue;

        if(working.compareAndSet(false, true)) {
            try {
                Future<Double> future = executor.submit(this);
                result = future.get(timeoutMillis, TimeUnit.MILLISECONDS);
            } catch (TimeoutException e) {
                // System.out.println("Timeout occurred, returning last value...");
            } catch (ExecutionException | InterruptedException e) {
                // e.printStackTrace();
            }
        }

        return result;
    }

    @Override
    public Double call() throws Exception {
        try {
            lastValue = sensor.getDistance(distanceUnit);
        } finally {
            working.set(false);
        }

        return lastValue;
    }

    public void shutdown() {
        executor.shutdown();
        try {
            // Wait a while for existing tasks to terminate
            if (!executor.awaitTermination(3, TimeUnit.SECONDS)) {
                executor.shutdownNow(); // Cancel currently executing tasks
                // Wait a while for tasks to respond to being cancelled
                if (!executor.awaitTermination(3, TimeUnit.SECONDS))
                    System.err.println("Pool did not terminate");
            }
        } catch (InterruptedException ie) {
            // (Re-)Cancel if current thread also interrupted
            executor.shutdownNow();
            // Preserve interrupt status
            Thread.currentThread().interrupt();
        }
    }
}
