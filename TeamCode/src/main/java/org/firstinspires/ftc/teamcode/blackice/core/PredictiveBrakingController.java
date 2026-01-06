package org.firstinspires.ftc.teamcode.blackice.core;

/**
 * A positional controller using proportional control plus a custom
 * velocity-based braking prediction model.
 * <p>
 * Instead of relying on gradual deceleration and a traditional derivative term, this
 * controller predicts the robot’s braking drift and subtracts it from the error. This
 * allows it to brake exactly when it needs to maximizing deceleration to
 * create a stronger, more responsive controller capable of precise high-speed stops.
 *
 * <p><b>Why:</b><br>
 * FTC drivetrains experience non-linear braking due to wheel slip.
 * Using a quadratic (v·|v|) + linear (v) damping model captures this behavior
 * more accurately than a standard PID derivative term.
 *
 * <p><b>Result:</b><br>
 * Zero-power-brake stopping performance with active correction,
 * typically ~5× faster deceleration than coasting.
 *
 * <p><b>Prerequisites:</b><br>
 * This controller assumes accurate velocity sensing even when powered wheels brake
 * and slide (e.g., dead wheels).
 *
 * <p><b>Tuning Procedure:</b><br>
 * 1. Determine kBraking (linear) and kFriction (quadratic) by measuring
 *    stopping distance at multiple velocities using zero-power-brake.
 * 2. Increase kP until the robot holds position firmly without oscillation.
 *
 * <p><b>Typical Values:</b><br>
 * – kBraking ≈ 0.07<br>
 * – kFriction ≈ 0.0015<br>
 * – kP ≈ 0.1–1.0 depending on drivetrain and weight
 * <p>
 * This controller is designed by FTC Team 18535, Frozen Code.
 * It is named "BlackIce" because the robot <b>slides</b> into position as if it were
 * navigating black ice.
 * <p>
 * <p><b>Does this make the robot use too much power when braking?</b><br>
 * Alternating full forward (+1) and full reverse (-1) power caused the control hub to restart due to low voltage spikes.
 * Solution: We capped the amount of voltage applied opposite to the direction of motion to be very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like zero-power brake mode, using the motor’s own momentum for braking without consuming significant energy.
 * However, too low will not have enough power to brake once it slows down after using
 * the motor’s own momentum for braking.
 *
 * @author Jacob Ophoven - 18535, Frozen Code
 */
public class PredictiveBrakingController {
    /**
     * Linear braking coefficient.
     * Models speed-proportional braking force (how much force the robot brakes with
     * relative to it's velocity).
     * Conceptually similar to a derivative damping term in a regular PID.
     */
    public double kBraking;
    
    /**
     * Quadratic friction/slippage coefficient.
     * Models non-linear wheel slippage from friction.
     */
    public double kFriction;
    
    /**
     * Proportional gain.
     * Controls how strongly the robot reacts to the remaining error after
     * subtracting predicted braking displacement.
     */
    public double kP;
    
    /**
     * Maximum amount of power applied in the opposite direction of momentum.
     * To high will cause the control hub to restart due to low voltage spikes.
     * Too low will not have enough power to brake once it slows down after using the
     * motor’s own momentum for braking.
     */
    public double maximumBrakingPower = 0.2;
    
    /**
     * Constructs the controller.
     *
     * @param kP                   proportional gain
     * @param kLinearBraking      linear damping coefficient
     * @param kQuadraticFriction  quadratic slippage/friction coefficient
     */
    public PredictiveBrakingController(double kP, double kLinearBraking,
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
        double directionOfMotion = Math.signum(velocity);
        
        double predictedBrakingDisplacement =
            directionOfMotion * velocity * velocity * kFriction + velocity * kBraking;

        double outputPower = kP * (error - predictedBrakingDisplacement);
        
        return clampReversePower(outputPower, directionOfMotion);
    }
    
    /**
     * Prevents the controller from applying too much power in the opposite direction of
     * the robot's momentum.
     * Alternating full forward (+1) and full reverse (-1) power caused the control hub to restart due to low voltage spikes.
     * This fixes it by capping the amount of voltage applied opposite to the
     * direction of motion to be very minimal. Even a tiny opposite voltage (e.g., -0.0001) locks the wheels like zero-power brake mode, using the motor’s own momentum for braking without consuming significant energy.
     */
    private double clampReversePower(double power, double directionOfMotion) {
        boolean isOpposingMotion = directionOfMotion * power < 0;
        if (!isOpposingMotion) {
            return power;
        }
        double clampedPower;
        if (power < 0) {
            clampedPower = Math.max(power, -maximumBrakingPower);
        } else {
            clampedPower = Math.min(power, maximumBrakingPower);
        }
        return clampedPower;
    }
}
