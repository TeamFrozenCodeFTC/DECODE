//package org.firstinspires.ftc.blackice.simple;
//
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.blackice.core.control.ErrorController;
//import org.firstinspires.ftc.blackice.core.hardware.drivetrain.Drivetrain;
//import org.firstinspires.ftc.blackice.core.hardware.localization.MotionState;
//import org.firstinspires.ftc.blackice.util.Logger;
//import org.firstinspires.ftc.blackice.util.geometry.Pose;
//import org.firstinspires.ftc.blackice.util.geometry.Vector;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//
//import java.util.function.Supplier;
//
///**
// * Uses error controllers to convert target velocities and positions into vectors and
// * motor powers to be driven.
// */
//public class DrivePowerController {
//    /**
//     * Responsible for turning the robot and making sure it is facing the correct
//     * direction.
//     */
//    public final PDController headingController;
//    /**
//     * Responsible holding a given pose and giving translational power for the robot to
//     * stay on the path. Tune this as aggressively as possible without the robot shaking
//     * while holding a pose. Error is in distance from target point (inches).
//     */
//    public final QuadraticDampedPIDController positionalController;
//
//    public final Drivetrain drivetrain;
//    private final Supplier<Double> voltageSupplier;
//
//    double lastDrivePower = 1;
//    boolean isBraking = false;
//    Vector brakingVector;
//
//    double totalError = 0;
//    double iterations = 0;
//
//    public DrivePowerController(PDController headingController,
//                                QuadraticDampedPIDController positionalController,
//                                Drivetrain drivetrain,
//                                Supplier<Double> voltageSupplier) {
//        this.headingController = headingController;
//        this.positionalController = positionalController;
//        this.drivetrain = drivetrain;
//        drivetrain.zeroPowerFloatMode();
//        this.voltageSupplier = voltageSupplier;
//    }
//
//    public void reset() {
//        headingController.reset();
//        positionalController.reset();
//        driveVelocityController.reset();
//        isBraking = false;
//        lastDrivePower = 1;
//        brakingVector = null;
//    }
//
//    public double getVoltage() {
//        return voltageSupplier.get();
//    }
//
//    /**
//     * A multiplier to scale power based on battery voltage.
//     * If tuned at 12V, and the current voltage is 11.5V, this will return 1.043
//     * to compensate for the lower voltage.
//     */
//    public double getVoltagePowerCompensation() {
//        return tunedVoltage / getVoltage();
//    }
//
//    /**
//     * Instructs the drivetrain to follow a vector relative to the field.
//     * <p>
//     * Includes voltage compensation and clamps reverse power to avoid brownouts.
//     */
//    public void followFieldVector(Vector drive, double turn, MotionState motionState) {
//        drivetrain.followVector(
//            motionState.makeRobotRelative(drive)
//                //.times(getVoltagePowerCompensation())
//                .map(motionState.robotRelativeVelocity, this::clampReversePower),
//            turn
//        );
//    }
//
//    public void holdPose(Pose pose, MotionState motionState) {
//        holdPose(pose, motionState, 1);
//    }
//
//    public void holdPose(Pose pose, MotionState motionState, double maxPower) {
//        Vector holdPower = computeHoldPower(pose.getPosition(), motionState).withMaxMagnitude(maxPower);
//
//        if (motionState.speed < 0.1 && holdPower.computeMagnitude() < 0.1) {
//            holdPower = new Vector(0,0);
//        }
//
//        double turnPower = Math.min(maxPower,
//            computeHeadingCorrectionPower(Math.toRadians(pose.getHeading()),
//                                          motionState));
//
//        if (motionState.angularVelocity < Math.toRadians(1) && Math.abs(turnPower) < 0.1) {
//            turnPower = 0;
//        }
//
//        followFieldVector(holdPower, turnPower, motionState);
//    }
//
//    public Vector computeHoldPower(Vector position, MotionState motionState) {
//        Vector error =
//            position.minus(motionState.position);
//        return error.map(motionState.velocity.times(-1),
//                         positionalController::runFromVelocity);
//    }
//
//    Vector computeCentripetalVector(PathState pathState) {
//        double centripetalForce =
//            pathState.tangentialVelocity * pathState.tangentialVelocity *
//                pathState.closestPathPoint.curvature * centripetalScaling;
//
//        return pathState.closestPathPoint.perpendicularVector.withMagnitude(
//            centripetalForce);
//    }
//
//    /**
//     * Computes the amount of feedforward velocity to reach the end of the path, while
//     * accounting for momentum.
//     */
//    private double computeMomentumCompensatedFeedforward(double targetVelocity,
//                                                         PathState pathState) {
//        // The velocity the robot would be at if it coasted to the end of the path.
//        double finalCoastVelocity = Math.sqrt(Math.abs(
//            pathState.tangentialVelocity * pathState.tangentialVelocity +
//                (2 * -naturalDeceleration *
//                    (pathState.closestPathPoint.distanceRemaining))));
//
//        // How much velocity the robot loses due to momentum while coasting to the end of the path.
//        double velocityToLoseDueToMomentum =
//            pathState.tangentialVelocity - finalCoastVelocity;
//
//        // We want to stop the robot's momentum so we subtract the
//        // velocityToLoseDueToMomentum from the target velocity.
//        // How much power the robot should add to be able to coast to the end target.
//        double addedFeedforwardVelocity =
//            targetVelocity - velocityToLoseDueToMomentum;
//
//        Logger.graph("momentumCompensatedFeedforward", addedFeedforwardVelocity);
//        Logger.graph("finalCoastVelocity", finalCoastVelocity);
//        Logger.graph("velocityToLoseDueToMomentum", velocityToLoseDueToMomentum);
//
//        if (pathState.path.behavior.allowReversePowerDeceleration) {
//            return addedFeedforwardVelocity;
//        }
//        return Math.max(0, addedFeedforwardVelocity);
//    }
//
//    Vector computeTangentialDriveVector(PathState pathState) {
//        double drivePower;
//        if (pathState.path.behavior.velocityProfile == null) {
//            drivePower = 1;
//            Logger.graph("targetPositionX", pathState.closestPathPoint.point.getX());
//            Logger.graph("currentPositionX", pathState.motionState.position.getX());
//        } else {
//            double targetVelocity =
//                pathState.path.behavior.velocityProfile.computeTargetVelocity(
//                    pathState.closestPathPoint.distanceAlongPath,
//                    pathState.closestPathPoint.distanceRemaining);
//
//            double momentumCompensatedFeedforward =
//                computeMomentumCompensatedFeedforward(targetVelocity, pathState);
//            double driveVelocityError = targetVelocity - pathState.tangentialVelocity;
//
//            Logger.graph("driveVelocityError", driveVelocityError);
//            Logger.graph("currentVelocity", pathState.tangentialVelocity);
//            Logger.graph("targetVelocity", targetVelocity);
//            Logger.graph("momentumCompensatedFeedforward", momentumCompensatedFeedforward);
//
//            drivePower = Range.clip(
//                driveVelocityController.computeCorrection(driveVelocityError,
//                                                          pathState.motionState.deltaTime)
//                    + driveVelocityController.computeFeedforward(momentumCompensatedFeedforward),
//                0, 1);
//        }
//        lastDrivePower = drivePower;
//        return pathState.closestPathPoint.tangent.withMagnitude(drivePower);
//    }
//
//    Vector computePerpendicularCorrectionVector(PathState pathState) {
//        Vector perpendicularDirection = pathState.closestPathPoint.perpendicularVector;
//
//        Vector error =
//            pathState.closestPathPoint.point.minus(pathState.motionState.nextPosition);
//
//        double perpendicularError = error.dotProduct(perpendicularDirection);
//        double perpendicularVelocity =
//            pathState.motionState.nextVelocity.dotProduct(perpendicularDirection);
//        double derivativeError = -perpendicularVelocity;
//        double translationalPower =
//            positionalController.runFromVelocity(perpendicularError, derivativeError);
//
//        Logger.graph("pathError", perpendicularError);
//        totalError += Math.abs(perpendicularError);
//        iterations++;
//
//        return perpendicularDirection.withMagnitude(translationalPower);
//    }
//
//    public double getAveragePathError() {
//        return totalError / iterations;
//    }
//
//    private static double consumeRemaining(double remaining, double used) {
//        return Math.max(0, remaining - used);
//    }
//
//    void drive(PathState pathState) {
//        if (isBraking) {
//            Logger.verbose("brakingPower", brakingVector);
//
//            holdPose(pathState.path.endPose, pathState.motionState);
//            return;
//        }
//
//        // Prioritizes
//        // 1. centripetal feedforward, to keep momentum on path curvature
//        // 2. braking translational, to not overshoot between path transitions
//        // 3. turning,
//        // 4. tangential drive power
//
//        // Translational is above turning because of its braking
//        // responsibility.
//        // All vectors have magnitudes <= 1
//
//        double remainingSq = 1.0;
//
//        Vector centripetalPower = new Vector(0, 0);
//        Vector translationalPower = new Vector(0, 0);
//        Vector drivePower = new Vector(0, 0);
//        double turnPower = 0;
//
//        if (useCentripetal) {
//            centripetalPower = computeCentripetalVector(pathState);
//            remainingSq =
//                consumeRemaining(remainingSq, centripetalPower.lengthSquared());
//        }
//        if (useTranslational && remainingSq > 0) {
//            translationalPower = computePerpendicularCorrectionVector(pathState);
//            remainingSq = consumeRemaining(remainingSq,
//                                           translationalPower.lengthSquared());
//        }
//        if (useHeading && remainingSq > 0) {
//            double targetHeading =
//                pathState.path.headingInterpolator.interpolate(pathState.closestPathPoint);
//            turnPower =
//                Range.clip(computeHeadingCorrectionPower(targetHeading,
//                                                         pathState.motionState), -1, 1);
//            double maxTurn = Math.sqrt(remainingSq);
//            turnPower = Math.copySign(Math.min(Math.abs(turnPower), maxTurn), turnPower);
//            remainingSq = consumeRemaining(remainingSq, turnPower * turnPower);
//        }
//        if (useDrive && remainingSq > 0) {
//            drivePower = computeTangentialDriveVector(pathState);
//            Vector driveResidual =
//                drivePower.minus(translationalPower.projectOnto(drivePower));
//            double driveMag = driveResidual.computeMagnitude();
//            double maxDriveMag = Math.min(driveMag, Math.sqrt(remainingSq));
//            drivePower = driveResidual.withMagnitude(maxDriveMag);
//        }
//
//        Vector totalPower = centripetalPower.plus(translationalPower).plus(drivePower);
//
//        Logger.verbose("centripetalPower", centripetalPower.computeMagnitude());
//        Logger.verbose("translationalPower", translationalPower.computeMagnitude());
//        Logger.verbose("turnPower", turnPower);
//        Logger.verbose("drivePower", drivePower.computeMagnitude());
//        Logger.verbose("totalPower", totalPower.computeMagnitude());
//        Logger.updateGraph();
//
//        Vector robotRelativePower = pathState.motionState.makeRobotRelative(totalPower)
//            .map(pathState.motionState.robotRelativeVelocity, this::clampReversePower);
//
//        drivetrain.followVector(robotRelativePower, turnPower);
//    }
//
//    public double computeHeadingCorrectionPower(double targetHeading,
//                                                MotionState motionState) {
//        double headingError = AngleUnit.RADIANS.normalize(targetHeading - motionState.heading);
//
////        Logger.verbose("targetHeading", Math.toDegrees(targetHeading));
////        Logger.verbose("currentHeading", Math.toDegrees(motionState.heading));
////        Logger.verbose("headingError", Math.toDegrees(headingError));
//
//        return headingController.computeCorrection(headingError, motionState.deltaTime)
//            + headingController.computeFeedforward(0);
//    }
//
//
//    void evaluateBraking(PathState pathState) {
//        brakingVector = computeHoldPower(pathState.path.endPose.getPosition(),
//                                         pathState.motionState);
//
//        boolean isNearEnd = pathState.closestPathPoint.distanceRemaining <= 20;
//        boolean lessThanDrive = brakingVector.computeMagnitude() < lastDrivePower;
//        boolean isReversing =
//            brakingVector.dotProduct(pathState.closestPathPoint.tangent) < 0;
//
//        isBraking = isBraking || (isNearEnd && (lessThanDrive || isReversing));
//    }
//
//    /**
//     * Prevents the robot from applying too much power in the opposite direction of the
//     * robot's momentum. This prevents voltage drops and doesn't hurt braking performance
//     * that much since reversing the direction of wheels with just -0.001 power behaves
//     * identical to -0.3 at high speeds due to EMF braking.
//     */
//    private double clampReversePower(double power, double directionOfMotion) {
//        boolean isOpposingMotion = directionOfMotion * power < 0;
//
//        if (!isOpposingMotion) {
//            return power;
//        }
//
//        double clampedPower;
//        if (power < 0) {
//            clampedPower = Math.max(power, -maxReversalBrakingPower);
//        } else {
//            clampedPower = Math.min(power, maxReversalBrakingPower);
//        }
//        return clampedPower;
//    }
//}
