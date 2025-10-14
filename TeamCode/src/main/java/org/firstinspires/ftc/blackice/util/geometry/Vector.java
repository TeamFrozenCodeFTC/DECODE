package org.firstinspires.ftc.blackice.util.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.blackice.util.OperableComponents;

/**
 * Immutable 2D vector class that represents drive vectors and robot
 * positions. This class is used for both field-relative and
 * robot-relative vectors.
 */
public class Vector extends OperableComponents<Vector> {
    /**
     * The unit vector that moves the robot forward when at 0
     * heading.
     */
    public static Vector FORWARD = new Vector(1, 0);
    /**
     * The unit vector that moves the robot forward when at 0
     * heading.
     */
    public static Vector BACKWARD = new Vector(-1, 0);
    /**
     * The unit vector that strafes the robot to the left when at 0
     * heading.
     */
    public static Vector LEFT = new Vector(0, 1);
    /**
     * The unit vector that strafes the robot to the right when at 0
     * heading.
     */
    public static Vector RIGHT = new Vector(0, -1);
    public static Vector ZERO = new Vector(0, 0);
    
    
    public Vector(double x, double y) {
        super(new double[]{x, y});
    }
    
    public Vector perpendicularLeft() {
        return new Vector(-getY(), getX());
    }
    
    public Vector mirroredAcrossYAxis() {
        // Field width is 144 inches
        return new Vector(144 - getX(), getY());
    }
    
    /**
     * Creates a vector with the same x and y values from a scalar.
     * <pre><code>
     * Vector.fromScalar(5) -> new Vector(5, 5)
     * </code></pre>
     */
    public static Vector fromScalar(double xAndYComponent) {
        return new Vector(xAndYComponent, xAndYComponent);
    }
    
    public Pose withHeading(double heading) {
        return new Pose(this, heading);
    }
    
    public double getX() {
        return this.components[0];
    }
    
    public double getY() {
        return this.components[1];
    }
    
    public Vector withX(double x) {
        return new Vector(x, this.getY());
    }
    
    public Vector withY(double y) {
        return new Vector(this.getX(), y);
    }
    
    public boolean equals(Vector obj) {
        if (this == obj) return true;
        
        return this.getX() == obj.getX() && this.getY() == obj.getY();
    }
    
    public static Vector fromPolar(double magnitude, double theta) {
        return new Vector(
            magnitude * Math.cos(theta),
            magnitude * Math.sin(theta)
        );
    }
    
    @SuppressLint("DefaultLocale")
    @NonNull
    public String toString() {
        return String.format("Vector{x=%.2f, y=%.2f}", getX(),
                             getY());
    }
    
    /**
     * Gives the magnitude of how much this vector is pointing in the
     * direction of the other vector.
     * <p>
     * Positive if in the same direction, negative if in the other
     * direction.
     */
    public double dotProduct(Vector other) {
        return this.getX() * other.getX() +
            this.getY() * other.getY();
    }
    
    /**
     * Rotates the vector counterclockwise by the given radians.
     */
    public Vector rotateCounterclockwiseBy(double radians) {
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        return new Vector(getX() * cos - getY() * sin,
                          getX() * sin + getY() * cos);
    }
    
    /**
     * Rotates the vector clockwise by the given radians.
     */
    public Vector rotateClockwiseBy(double radians) {
        return rotateCounterclockwiseBy(-radians);
    }
    
    /**
     * Computes forward and lateral distances from the robot to a
     * target point.
     *
     * @param robotPosition current robot position (x, y)
     * @param robotHeading  current robot heading (in radians)
     * @param targetPoint   target (x, y) point
     * @return A Vector where: x = forward distance, y = lateral
     * (strafe) distance
     */
    public static Vector calculateForwardAndLateralOffset(
        Vector robotPosition,
        double robotHeading,
        Vector targetPoint
    ) {
        Vector toTarget = targetPoint.minus(robotPosition);
        return toTarget.rotateCounterclockwiseBy(-robotHeading);
    }
    
    public double computeMagnitude() {
        return Math.hypot(getX(), getY());
    }
    
    public double lengthSquared() {
        return getX() * getX() + getY() * getY();
    }
    
    public Vector projectOnto(Vector other) {
        double otherLenSq = other.lengthSquared();
        if (otherLenSq < 1e-12) {
            // Avoid division by zero if other is zero vector
            return new Vector(0, 0);
        }
        double scaleFactor = this.dotProduct(other) / otherLenSq;
        return other.times(scaleFactor);
    }
    
    public double distanceTo(Vector point) {
        double dx = point.getX() - this.getX();
        double dy = point.getY() - this.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    public Vector withMagnitude(double newMagnitude) {
        double magnitude = computeMagnitude();
        return this.divide(magnitude).times(newMagnitude);
    }
    
    public Vector withMagnitude(double magnitude, double newMagnitude) {
        return this.divide(magnitude).times(newMagnitude);
    }
    
    public Vector withMaxMagnitude(double maxMagnitude) {
        double magnitude = computeMagnitude();
        if (magnitude <= maxMagnitude) {
            return this;
        }
        return this.divide(magnitude).times(maxMagnitude);
    }
    
    public Vector withMinMagnitude(double minMagnitude) {
        double magnitude = computeMagnitude();
        if (magnitude > minMagnitude) {
            return this;
        }
        return this.divide(magnitude).times(minMagnitude);
    }
    
    public Vector normalized() {
        return withMagnitude(1);
    }
    
    public double getAngleToLookAt(Vector point) {
        Vector direction = this.minus(point);
        return Math.atan2(direction.getY(), direction.getX());
    }
    
    public Vector abs() {
        return map(Math::abs);
    }
    
    /**
     * Returns the angle of this vector in degrees, counterclockwise
     * from the positive X-axis.
     */
    public double calculateAngle() {
        return Math.toDegrees(calculateRadians());
    }
    
    public double calculateRadians() {
        return Math.atan2(getY(), getX());
    }
    
    @Override
    protected Vector fromComponentArray(double[] components) {
        return new Vector(components[0], components[1]);
    }
}
