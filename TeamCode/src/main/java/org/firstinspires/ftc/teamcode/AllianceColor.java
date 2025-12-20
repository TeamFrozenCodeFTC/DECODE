package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

public enum AllianceColor {
    BLUE(new Vector(3.5, 144-3.5),
         new Pose(144-16.5/2, 17.0/2, 0),
         new Pose(18, 121, -36)),
    RED(BLUE.goalPosition.mirroredAcrossYAxis(),
        BLUE.humanResetZone.mirroredAcrossYAxis(),
        BLUE.goalReset.mirroredAcrossYAxis());

    private final Vector goalPosition;
    private final Pose humanResetZone;
    private final Pose goalReset;

    AllianceColor(Vector goalPosition, Pose humanPlayerZone, Pose goalReset) {
        this.goalPosition = goalPosition;
        this.humanResetZone = humanPlayerZone;
        this.goalReset = goalReset;
    }

    public Vector getGoalPosition() {
        return goalPosition;
    }

    public Pose getHumanResetZone() {
        return humanResetZone;
    }
    
    public Pose getGoalReset() {
        return goalReset;
    }
}
