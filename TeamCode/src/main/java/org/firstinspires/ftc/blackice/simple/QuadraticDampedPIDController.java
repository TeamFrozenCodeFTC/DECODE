package org.firstinspires.ftc.blackice.simple;

/**
 * A positional controller with a proportional term and a special “derivative”
 * term modeled as a combination of linear and quadratic damping.
 * <p>
 * The key idea: FTC drivetrains experience non-linear friction from wheel
 * slippage when braking at higher speeds. Instead of relying on gradual
 * deceleration and a traditional derivative term, this controller predicts how far it
 * takes the robot to brake based on its current velocity. That predicted drift is
 * subtracted from the error so the robot brakes exactly when it needs to and precisely
 * corrects into place.
 * <p>
 * This controller was designed with dead wheel localization, where velocity is
 * measured accurately even when powered wheels brake and slide.
 *
 * <p><b>Tuning procedure:</b><br>
 * 1. Determine the linear (kBraking) and quadratic (kFriction) damping
 *    coefficients experimentally by measuring stopping distance at various
 *    velocities with motors set to zero-power brake.
 *    These values naturally include system delay and reflect your robot’s
 *    fastest achievable braking response.<br>
 * 2. Increase kP until the robot holds position firmly without oscillations.
 * <p>
 * Typical values:<br>
 * – kBraking ≈ 0  .07<br>
 * – kFriction ≈ 0.0015<br>
 * – kP ≈ 0.1–1.0 depending on drivetrain and weight
 * <p>
 * This controller is designed by FTC Team 18535, Frozen Code.
 * It is named "BlackIceController" because the robot <b>slides</b> into position like it's
 * navigating black ice.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public class QuadraticDampedPIDController {
    /**
     * Linear braking coefficient.
     * Models speed-proportional braking force (How much force the robot brakes with)
     * Conceptually similar to a first-derivative damping term.
     */
    public double kBraking;
    
    /**
     * Quadratic friction/slippage coefficient.
     * Models non-linear wheel slippage and friction.
     */
    public double kFriction;
    
    /**
     * Proportional gain.
     * Controls how strongly the robot reacts to the remaining error after
     * subtracting predicted drift.
     */
    public double kP;
    
    /**
     * Constructs the controller.
     *
     * @param kP                   proportional gain
     * @param kLinearBraking      linear damping coefficient
     * @param kQuadraticFriction  quadratic slippage/friction coefficient
     */
    public QuadraticDampedPIDController(double kP, double kLinearBraking,
                                        double kQuadraticFriction) {
        this.kP = kP;
        this.kBraking = kLinearBraking;
        this.kFriction = kQuadraticFriction;
    }
    
    /**
     * Computes the control output.
     * <p>
     * Predicts braking displacement using:
     *   velocity * |velocity| * kFriction   // quadratic slippage
     * + velocity * kBraking                 // linear braking
     * <p>
     * Then subtracts this prediction from the error and scales by kP.
     * <p>
     * @param error     current positional error
     * @param velocity  current velocity
     * @return          control output (e.g., motor power)
     */
    public double computeOutput(double error, double velocity) {
        double predictedBrakingDisplacement =
            velocity * Math.abs(velocity) * kFriction + velocity * kBraking;
        
        return kP * (error - predictedBrakingDisplacement);
    }
}
