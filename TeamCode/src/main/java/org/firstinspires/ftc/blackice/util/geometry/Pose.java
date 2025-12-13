package org.firstinspires.ftc.blackice.util.geometry;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.blackice.util.Utils;

import java.util.Objects;

public class Pose {
    private final Vector position;
    private final @Nullable Double heading;

    public Pose(Vector position, @Nullable Double heading) {
        this.position = position;
        this.heading = heading;
    }
    
    public Pose(double x, double y) {
        this(new Vector(x, y), null);
    }
    
    public Pose(double x, double y, double heading) {
        this(new Vector(x, y), heading);
    }
    
    public static Pose toDegrees(double x, double y, double heading) {
        return new Pose(new Vector(x, y), Math.toDegrees(heading));
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
    
    public Pose completeWith(Pose pose) {
        return new Pose(
            this.position,
            Utils.getOrDefault(heading, pose.getHeading())
        );
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
