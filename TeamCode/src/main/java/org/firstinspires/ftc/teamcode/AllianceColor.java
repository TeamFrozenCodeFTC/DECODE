package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.blackice.geometry.Pose;
import org.firstinspires.ftc.teamcode.blackice.geometry.Vector;

public enum AllianceColor {
    // 16.5, 17.66
    
    // (144, 144) - start (0, 0)
    BLUE(new Vector(3.5, 144-3.5-2),
         new Pose(144-17.75/2, 8, 0),
         new Pose(18.85, 119.32, -36)),
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
