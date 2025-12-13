package org.firstinspires.ftc;

public class PIDController {
    public double kP, kI, kD;
    public double iZone;  // Integral allowed only when |error| < iZone
    
    private static final double alpha = 0.01;
    private double filteredVoltage;
    private double previousError;
    private double integral;
    private boolean firstRun = true;
    
    public PIDController(double kP, double kI, double kD, double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iZone = iZone;
    }
    
    public double computeOutput(double error, double deltaTime, double voltage) {
        double derivative;
        if (firstRun) {
            derivative = 0;
            firstRun = false;
        } else {
            derivative = (kD != 0 && deltaTime > 1e-6)
                ? (error - previousError) / deltaTime
                : 0;
        }
        
        if (Math.abs(error) < iZone) {
            integral += error * deltaTime;
            filteredVoltage = filteredVoltage + alpha * (voltage - filteredVoltage);
        }
        
        double output = (kP * error
                + kI * integral
                + kD * derivative) / filteredVoltage;
        
        previousError = error;
        return output;
    }
    
    public void reset() {
        firstRun = true;
        previousError = 0;
        integral = 0;
    }
}
