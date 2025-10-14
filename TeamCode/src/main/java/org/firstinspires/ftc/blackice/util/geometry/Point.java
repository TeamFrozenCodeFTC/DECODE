package org.firstinspires.ftc.blackice.util.geometry;

/**
 * Represents a point (absolute location) on the field.
 * Extends Vector so you can still use all vector math operations.
 */
public class Point extends Vector {
    public Point(double x, double y) {
        super(x, y);
    }

    // Optional: factory method for readability
    public static Point at(double x, double y) {
        return new Point(x, y);
    }
}
