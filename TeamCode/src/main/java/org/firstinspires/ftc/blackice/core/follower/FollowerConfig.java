package org.firstinspires.ftc.blackice.core.follower;

import org.firstinspires.ftc.blackice.core.control.ErrorController;
import org.firstinspires.ftc.blackice.core.hardware.drivetrain.DrivetrainConfig;
import org.firstinspires.ftc.blackice.core.control.QuadraticDampedPIDController;
import org.firstinspires.ftc.blackice.core.hardware.localization.localizers.LocalizerConfig;
import org.firstinspires.ftc.blackice.core.paths.PathBehavior;
import org.firstinspires.ftc.blackice.util.Validator;

/**
 * Configuration for a Follower. DefaultFollower -> Follower (OpMode) -> Path
 */
public class FollowerConfig {
    LocalizerConfig localizerConfig;
    DrivetrainConfig drivetrainConfig;
    ErrorController headingPID;
    ErrorController positionalPID;
    ErrorController driveVelocityPIDF;
    PathBehavior defaultPathBehavior = new PathBehavior();
    
    double centripetalFeedforward = 0.005;
    double maxReversalBrakingPower = 0.3;
    double stopIfVoltageBelow = 7;
    double naturalDeceleration = 40;
    double tunedVoltage = 12.0;
    
    public FollowerConfig tunedVoltage(double tunedVoltage) {
        this.tunedVoltage = Validator.ensurePositiveSign(tunedVoltage);
        return this;
    }
    
    /**
     * The natural deceleration of your robot (in/s^2).
     * Should be positive. Used for momentum damping.
     * Only needed for smooth deceleration with velocity profiles.
     */
    public FollowerConfig naturalDeceleration(double naturalDeceleration) {
        this.naturalDeceleration = Validator.ensurePositiveSign(naturalDeceleration);
        return this;
    }
    
    public FollowerConfig stopIfVoltageBelow(double stopIfVoltageBelow) {
        this.stopIfVoltageBelow = Validator.ensurePositiveSign(stopIfVoltageBelow);
        return this;
    }
    
    /**
     * Sets the default behavior for every path used by this follower.
     */
    public FollowerConfig defaultPathBehavior(PathBehavior defaultPathBehavior) {
        this.defaultPathBehavior = defaultPathBehavior;
        return this;
    }
    
    /**
     * The localizer used to determine the robot's position on the field.
     */
    public FollowerConfig localizerConfig(LocalizerConfig localizerConfig) {
        this.localizerConfig = localizerConfig;
        return this;
    }
    
    /**
     * The drivetrain used to drive the robot.
     */
    public FollowerConfig drivetrainConfig(DrivetrainConfig drivetrainConfig) {
        this.drivetrainConfig = drivetrainConfig;
        return this;
    }
    
    /**
     * The controller responsible for turning the robot and making sure it is facing
     * the correct direction. Error is in radians.
     * Recommended starting values: headingPID(2, 0, 0.1)
     */
    public FollowerConfig headingPID(ErrorController headingPID) {
        this.headingPID = headingPID;
        return this;
    }
    
    /**
     * Responsible holding a given pose and giving translational power for the robot to
     * stay on the path. Tune this as aggressively as possible without the robot shaking
     * while holding a pose. Error is in distance from target point (inches).
     */
    public FollowerConfig positionalPID(QuadraticDampedPIDController positionalPID) {
        this.positionalPID = positionalPID;
        return this;
    }
    
    /**
     * Responsible accelerating and decelerating the robot to the target velocity.
     * Error is in inches per second.
     * <p>
     * Is only needed to follow velocity profiles and velocity constraints. Not necessary
     * for maximum robot speed.
     */
    public FollowerConfig driveVelocityController(
        ErrorController driveVelocityPIDF) {
        this.driveVelocityPIDF = driveVelocityPIDF;
        return this;
    }
    
    /**
     * The max amount of power the robot can apply in the opposite direction to the
     * robot's motion. The robot will not be able to stop if this is <= 0. Warning: the
     * higher this is the more voltage will drop from the robot, resulting in unexpected
     * robot crashes.
     * <p>
     * Should be around 0.2 to 0.5.
     */
    public FollowerConfig maxReversalBrakingPower(double maxReversalBrakingPower) {
        this.maxReversalBrakingPower =
            Validator.ensurePositiveSign(maxReversalBrakingPower);
        return this;
    }
    
    public FollowerConfig centripetalFeedforward(double centripetalFeedforward) {
        this.centripetalFeedforward = centripetalFeedforward;
        return this;
    }
}
