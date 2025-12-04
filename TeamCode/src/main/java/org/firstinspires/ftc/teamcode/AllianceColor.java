package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.blackice.util.geometry.Pose;
import org.firstinspires.ftc.blackice.util.geometry.Vector;

public enum AllianceColor {
    // 16.5 is average of robot lengths (16x17)
    BLUE(new Vector(6, 144-6), new Vector(105.25, 33.25),
         new Pose(135.25, 8.5, 0)),
    RED(BLUE.goalPosition.mirroredAcrossYAxis(),
        BLUE.basePosition.mirroredAcrossYAxis(), new Pose(8.25, 8.5, 180));
    
    private final Vector goalPosition;
    private final Vector basePosition;
    private final Pose humanPlayerZone;
    
    AllianceColor(Vector goalPosition, Vector basePosition, Pose humanPlayerZone) {
        this.goalPosition = goalPosition;
        this.basePosition = basePosition;
        this.humanPlayerZone = humanPlayerZone;
    }

    public Vector getGoalPosition() {
        return goalPosition;
    }
    
    public Vector getBasePosition() {
        return basePosition;
    }
    
    public Pose getHumanPlayerZone() {
        return humanPlayerZone;
    }
}
