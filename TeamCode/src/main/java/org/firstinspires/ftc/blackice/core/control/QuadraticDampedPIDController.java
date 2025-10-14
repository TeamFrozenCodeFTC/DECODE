package org.firstinspires.ftc.blackice.core.control;

/**
 * A PD controller with quadratic damping **quadratic-damped PIDs** to model the non-linear effects of friction and momentum.but more empirical. Also has basically a
 * Derivative of
 * Derivative term or velocity^2
 * term to help stop more smoothly because EMF braking is not perfectly quadratic or linear.
 * Also is predictive because D term compensates for predictedPosition = position + velocity * deltaTime.
 */
public class QuadraticDampedPIDController extends PIDFController {
    public double kBraking;
    public double kFriction;
    public double kStaticFriction;
    
    public QuadraticDampedPIDController(double kP, double kLinearBraking,
                                        double kQuadraticFriction, double kStaticFriction) {
        super(kP, 0, 0, Feedforward.zero());
        this.kBraking = kLinearBraking;
        this.kFriction = kQuadraticFriction;
        this.kStaticFriction = kStaticFriction;
    }
    
    @Override
    public double runFromVelocity(double error, double derivative) {
        double velocity = -derivative;
        double predictedBrakingDisplacement = velocity * Math.abs(velocity) * kFriction + velocity *
            kBraking + kStaticFriction * Math.signum(velocity);
        return super.runFromVelocity(
            error - predictedBrakingDisplacement,
            derivative
        );
    }
}
