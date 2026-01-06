package org.firstinspires.ftc.teamcode.blackice.core;

import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public class MotionTolerance {
    public final double linearVelocity;   // in/s
    public final double angularVelocity;  // deg/s

    public MotionTolerance(double linearVelocity, double angularVelocity) {
        this.linearVelocity = linearVelocity;
        this.angularVelocity = angularVelocity;
    }

    public boolean isStopped(Vector currentVelocity, double angularVelocity) {
        return currentVelocity.computeMagnitude() <= linearVelocity
            && Math.toDegrees(Math.abs(angularVelocity)) <= this.angularVelocity;
    }
}
