package org.firstinspires.ftc.teamcode.miniblackice.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import java.util.Objects;

public class Pose {
    private final Vector position;
    private final @Nullable Double heading;
    
    /**
     * Creates a Pose with position and heading in degrees. (Internally uses radians)
     */
    public Pose(Vector position, @Nullable Double heading) {
        this.position = position;
        this.heading = heading == null ? null : Math.toRadians(heading);
    }
    
    public Pose(double x, double y) {
        this(new Vector(x, y), null);
    }
    
    public Pose(double x, double y, double heading) {
        this(new Vector(x, y), heading);
    }
    
    public Pose addedX(double x) {
        return new Pose(position.withX(position.getX() + x), heading);
    }
    
    public Pose addedY(double y) {
        return new Pose(position.withY(position.getY() + y), heading);
    }
    
    public Pose addedHeading(double heading) {
        return new Pose(position,
                        getHeading() + heading);
    }
    
    public Pose headingToDegrees() {
        return this.withHeading(Math.toDegrees(getHeading()));
    }
    
    public Vector getPosition() {
        return position;
    }
    
    public boolean hasHeading() {
        return heading != null;
    }
    
    public Pose mirroredAcrossYAxis() {
        return new Pose(144 - position.getX(), position.getY(), heading == null ? null :
            180 - heading);
    }
    
    public @NonNull Double getHeading() {
        return Objects.requireNonNull(heading, "heading is null");
    }
    
    public @Nullable Double getNullableHeading() {
        return heading;
    }
    
    /**
     * Change the internal heading value (in radians).
     */
    public Pose withHeading(double heading) {
        return new Pose(position, heading);
    }
    
    public Pose withPosition(Vector position) {
        return new Pose(position, heading);
    }
    
    public Pose withX(double x) {
        return new Pose(position.withX(x), heading);
    }
    
    public Pose withY(double y) {
        return new Pose(position.withY(y), heading);
    }
    
    @SuppressLint("DefaultLocale")
    @NonNull
    public String toString() {
        return String.format("Pose{x=%.2f,y=%.2f,h=%.2f}", position.getX(),
                             position.getY(),
                             heading == null ? null :
                                 Math.toDegrees(heading));
    }
}
