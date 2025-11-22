package org.firstinspires.ftc.blackice.core.control;

/**
 * Represents a PIDF (Proportional-Integral-Derivative-Feedforward) controller.
 */
public class PIDFController implements ErrorController {
    public double kP, kI, kD;
    public Feedforward feedforward;
    
    private double previousError, integralSum;
    private boolean firstRun = true;
    
    /**
     * Creates a simple velocity controller of a proportional and feedforward based
     * on kV and kS (velocity and static gain).
     * <p>
     * Does not use Integral or Derivative terms as they are usually unnecessary
     * for this controller and can often add instability unless tuned appropriately.
     * Uses momentum damping based off the robot's natural deceleration instead of {@code
     * kA} (acceleration gain).
     *
     * @param kP the proportional gain, how much power the robot reacts to error.
     *          Usually around 0.01.
     * @param kV the velocity gain, how much power the robot requires per
     *                  velocity. Should be around 1/maxVelocity or 1/65 ~= 0.015
     * @param kS the static gain, the minimum power required to start moving.
     *                Should be determined experimentally. Should be around 0.05.
     */
    public PIDFController(double kP, double kI, double kD, double kS, double kV) {
        this(kP, kI, kD, new Feedforward.Linear(kV, kS));
    }
    
    public PIDFController(double kP, double kI, double kD, Feedforward feedforward) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;  
        this.feedforward = feedforward;
    }
    
    public void setCoefficients(double kP, double kI, double kD, double kS, double kV) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.feedforward = new Feedforward.Linear(kV, kS);
    }

    /** Run using a known error and its velocity as the derivative (derivative =
     * -velocity) */
    public double runFromVelocity(double error, double velocity) {
        return computeOutput(error, -velocity);
    }
    
    /** Run using only error and deltaTime, computes derivative internally */
    @Override
    public double computeCorrection(double error, double deltaTime) {
        double derivative;
        if (firstRun) {
            derivative = 0;
            firstRun = false;
        }
        else {
            derivative = (kD != 0 && deltaTime > 1e-6) ?
                (error - previousError) / deltaTime : 0;
        }
        
        return computeOutput(error, derivative);
    }
    
    private double computeOutput(double error, double derivative) {
        double output = kP * error;
        
        if (kI != 0) {
            integralSum += error;
            output += kI * integralSum;
        }
        if (kD != 0) {
            output += kD * derivative;
        }
        
        previousError = error;
        return output;
    }
    
    @Override
    public double computeFeedforward(double target) {
        return feedforward.compute(target);
    }
    
    @Override
    public void reset() {
        firstRun = true;
        previousError = 0;
        integralSum = 0;
    }
}
