package org.firstinspires.ftc.teamcode.blackice.geometry;


public class Vector {
    private final double x;
    private final double y;
    
    public Vector(double x, double y) {
        this.x = x;
        this.y = y;
    }
    
    public static Vector fromPolar(double magnitude, double angle) {
        return new Vector(
            magnitude * Math.cos(angle),
            magnitude * Math.sin(angle)
        );
    }
    
    public Vector mirroredAcrossYAxis() {
        return new Vector(144 - getX(), getY());
    }

    public double getX() { return x; }
    public double getY() { return y; }

    public Vector plus(Vector other) {
        return new Vector(this.x + other.x, this.y + other.y);
    }
    
    public Vector minus(Vector other) {
        return new Vector(this.x - other.x, this.y - other.y);
    }
    
    public Vector times(double scalar) {
        return new Vector(this.x * scalar, this.y * scalar);
    }
    
    public Vector dividedBy(double scalar) {
        return new Vector(this.x / scalar, this.y / scalar);
    }
    
    public double computeMagnitude() {
        return Math.hypot(x, y);
    }
    
    public Vector withMagnitude(double magnitude) {
        double current = computeMagnitude();
        if (current == 0) return new Vector(0, 0);
        return times(magnitude / current);
    }
    
    public Vector withMaxMagnitude(double maxMagnitude) {
        double m = computeMagnitude();
        if (m > maxMagnitude) return withMagnitude(maxMagnitude);
        return this;
    }

    public Vector map(Vector other,
                      BiFunctionDouble mappingFunction) {
        return new Vector(
            mappingFunction.apply(getX(), other.getX()),
            mappingFunction.apply(getY(), other.getY())
        );
    }
    
    public double getAngle() {
        return Math.atan2(getY(), getX());
    }
    
    /**
     * Apply a custom operation to each pair of components.
     * <code>new Vector(operation(x1, x2), operation(y1, y2))</code>
     */
    @FunctionalInterface
    public interface BiFunctionDouble {
        double apply(double component1, double component2);
    }
    
    public double dot(Vector other) {
        return this.x * other.x + this.y * other.y;
    }
    
    public double cross(Vector other) {
        return this.x * other.y - this.y * other.x;
    }
    
    public Vector withX(double newX) {
        return new Vector(newX, this.y);
    }
    
    public Vector withY(double newY) {
        return new Vector(this.x, newY);
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
    
    public double getAngleToLookAt(Vector point) {
        Vector direction = this.minus(point);
        return Math.atan2(direction.getY(), direction.getX());
    }
    
    public double distanceTo(Vector point) {
        double dx = point.getX() - this.getX();
        double dy = point.getY() - this.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }
    
    public Vector perpendicularLeft() {
        return new Vector(-getY(), getX());
    }
    
    public double lengthSquared() {
        return getX() * getX() + getY() * getY();
    }
    
    public Vector normalized() {
        return withMagnitude(1);
    }
    
    @Override
    public String toString() {
        return "Vector(" + x + ", " + y + ")";
    }
}
