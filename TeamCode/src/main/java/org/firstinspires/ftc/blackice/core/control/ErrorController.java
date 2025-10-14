package org.firstinspires.ftc.blackice.core.control;

public interface ErrorController {
    default double run(double target, double current, double deltaTime) {
        return computeCorrection(target - current, deltaTime) + computeFeedforward(target);
    };

    double runFromVelocity(double error, double velocity);
    
    double computeCorrection(double error, double deltaTime);
    
    double computeFeedforward(double target);
    
    void reset();
}
