package org.firstinspires.ftc.teamcode.blackice.core;

public class PDController {
    public double kP, kD;

    private double previousError;
    private boolean firstRun = true;
    
    public PDController(double kP, double kD) {
        this.kP = kP;
        this.kD = kD;
    }
    
    public double computeOutput(double error, double deltaTime) {
        double derivative;
        if (firstRun) {
            derivative = 0;
            firstRun = false;
        } else {
            derivative = (kD != 0 && deltaTime > 1e-6)
                ? (error - previousError) / deltaTime
                : 0;
        }
        
        double output = kP * error + kD * derivative;
        
        previousError = error;
        return output;
    }
    
    public void reset() {
        firstRun = true;
        previousError = 0;
    }
}
