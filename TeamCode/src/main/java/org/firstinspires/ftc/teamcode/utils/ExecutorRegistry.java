package org.firstinspires.ftc.teamcode.utils;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.TimeUnit;

public class ExecutorRegistry {
    private static final List<ExecutorService> executors = new ArrayList<>();

    public static ExecutorService register(ExecutorService executor) {
        executors.add(executor);
        return executor;
    }

    public static void shutdownAll() {
        for(ExecutorService executor : executors) {
            executor.shutdown();
            try {
                if (!executor.awaitTermination(3, TimeUnit.SECONDS)) {
                    executor.shutdownNow();
                }
            } catch (InterruptedException ie) {
                executor.shutdownNow();
                Thread.currentThread().interrupt();
            }
        }
        executors.clear();
    }
}
