package org.firstinspires.ftc.teamcode.blackice.core.commands;

public abstract class Command {
    private static final double DEFAULT_TIMEOUT = 99; // default 5 seconds
    private final double timeoutSeconds;
    private double startTime = 0;
    
    public Command() {
        this.timeoutSeconds = DEFAULT_TIMEOUT;
    }
    
    public static Command singleAction(Runnable action) {
        return new Command() {
            @Override
            protected void onStart() {
                action.run();
            }
            
            @Override
            public void update() {
            
            }
            
            @Override
            protected boolean onIsFinished() {
                return true;
            }
        };
    }
    
    public Command(double timeoutSeconds) {
        this.timeoutSeconds = timeoutSeconds;
    }
    
    public final void start() {
        startTime = now();
        onStart();
        update();
    }
    
    public final boolean isFinished() {
        return onIsFinished() || now() - startTime >= timeoutSeconds;
    }
    
    protected abstract void onStart();
    public abstract void update();
    protected abstract boolean onIsFinished();

    private double now() {
        return System.nanoTime() * 1e-9;
    }
}
